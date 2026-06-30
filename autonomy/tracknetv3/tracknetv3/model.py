"""TrackNetV3 model definitions adapted from the MIT-licensed upstream project."""

import torch
import torch.nn as nn


class Conv2DBlock(nn.Module):
    """Convolution, batch normalization, and ReLU block."""

    def __init__(self, in_dim: int, out_dim: int):
        super().__init__()
        self.conv = nn.Conv2d(in_dim, out_dim, kernel_size=3, padding="same", bias=False)
        self.bn = nn.BatchNorm2d(out_dim)
        self.relu = nn.ReLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.conv(x)
        x = self.bn(x)
        return self.relu(x)


class Double2DConv(nn.Module):
    """Two stacked 2D convolution blocks."""

    def __init__(self, in_dim: int, out_dim: int):
        super().__init__()
        self.conv_1 = Conv2DBlock(in_dim, out_dim)
        self.conv_2 = Conv2DBlock(out_dim, out_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.conv_1(x)
        return self.conv_2(x)


class Triple2DConv(nn.Module):
    """Three stacked 2D convolution blocks."""

    def __init__(self, in_dim: int, out_dim: int):
        super().__init__()
        self.conv_1 = Conv2DBlock(in_dim, out_dim)
        self.conv_2 = Conv2DBlock(out_dim, out_dim)
        self.conv_3 = Conv2DBlock(out_dim, out_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.conv_1(x)
        x = self.conv_2(x)
        return self.conv_3(x)


class TrackNet(nn.Module):
    """TrackNet heatmap prediction network."""

    def __init__(self, in_dim: int, out_dim: int):
        super().__init__()
        self.down_block_1 = Double2DConv(in_dim, 64)
        self.down_block_2 = Double2DConv(64, 128)
        self.down_block_3 = Triple2DConv(128, 256)
        self.bottleneck = Triple2DConv(256, 512)
        self.up_block_1 = Triple2DConv(768, 256)
        self.up_block_2 = Double2DConv(384, 128)
        self.up_block_3 = Double2DConv(192, 64)
        self.predictor = nn.Conv2d(64, out_dim, (1, 1))
        self.sigmoid = nn.Sigmoid()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x1 = self.down_block_1(x)
        x = nn.MaxPool2d((2, 2), stride=(2, 2))(x1)
        x2 = self.down_block_2(x)
        x = nn.MaxPool2d((2, 2), stride=(2, 2))(x2)
        x3 = self.down_block_3(x)
        x = nn.MaxPool2d((2, 2), stride=(2, 2))(x3)
        x = self.bottleneck(x)
        x = torch.cat([nn.Upsample(scale_factor=2)(x), x3], dim=1)
        x = self.up_block_1(x)
        x = torch.cat([nn.Upsample(scale_factor=2)(x), x2], dim=1)
        x = self.up_block_2(x)
        x = torch.cat([nn.Upsample(scale_factor=2)(x), x1], dim=1)
        x = self.up_block_3(x)
        x = self.predictor(x)
        return self.sigmoid(x)


class Conv1DBlock(nn.Module):
    """Convolution and LeakyReLU block."""

    def __init__(self, in_dim: int, out_dim: int):
        super().__init__()
        self.conv = nn.Conv1d(in_dim, out_dim, kernel_size=3, padding="same", bias=True)
        self.relu = nn.LeakyReLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.conv(x)
        return self.relu(x)


class Double1DConv(nn.Module):
    """Two stacked 1D convolution blocks."""

    def __init__(self, in_dim: int, out_dim: int):
        super().__init__()
        self.conv_1 = Conv1DBlock(in_dim, out_dim)
        self.conv_2 = Conv1DBlock(out_dim, out_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.conv_1(x)
        return self.conv_2(x)


class InpaintNet(nn.Module):
    """Trajectory rectification network."""

    def __init__(self):
        super().__init__()
        self.down_1 = Conv1DBlock(3, 32)
        self.down_2 = Conv1DBlock(32, 64)
        self.down_3 = Conv1DBlock(64, 128)
        self.bottleneck = Double1DConv(128, 256)
        self.up_1 = Conv1DBlock(384, 128)
        self.up_2 = Conv1DBlock(192, 64)
        self.up_3 = Conv1DBlock(96, 32)
        self.predictor = nn.Conv1d(32, 2, 3, padding="same")
        self.sigmoid = nn.Sigmoid()

    def forward(self, x: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
        x = torch.cat([x, mask], dim=2)
        x = x.permute(0, 2, 1)
        x1 = self.down_1(x)
        x2 = self.down_2(x1)
        x3 = self.down_3(x2)
        x = self.bottleneck(x3)
        x = torch.cat([x, x3], dim=1)
        x = self.up_1(x)
        x = torch.cat([x, x2], dim=1)
        x = self.up_2(x)
        x = torch.cat([x, x1], dim=1)
        x = self.up_3(x)
        x = self.predictor(x)
        x = self.sigmoid(x)
        return x.permute(0, 2, 1)


def create_tracknet(seq_len: int, bg_mode: str) -> TrackNet:
    """Create TrackNet with the input channel layout used by a checkpoint."""
    if bg_mode == "subtract":
        return TrackNet(in_dim=seq_len, out_dim=seq_len)
    if bg_mode == "subtract_concat":
        return TrackNet(in_dim=seq_len * 4, out_dim=seq_len)
    if bg_mode == "concat":
        return TrackNet(in_dim=(seq_len + 1) * 3, out_dim=seq_len)
    return TrackNet(in_dim=seq_len * 3, out_dim=seq_len)
