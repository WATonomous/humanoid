#!/bin/bash
FIXUID=$(id -u)
FIXGID=$(id -g)
BASE_PORT=${BASE_PORT:-$(($(id -u)*20))}
GUI_TOOLS_VNC_PORT=${GUI_TOOLS_VNC_PORT:-$((BASE_PORT++))}
WANDB_MODE="online"
key=""

# WandB setup
while true; do
    read -p "Do you wish to use Weights and Biases for Nerf PyTorch? [Y/n] " yn
    case $yn in
        [Yy]* ) WANDB_MODE="online"; read -p "Enter your WandB API key: " key ; break;;
        [Nn]* ) WANDB_MODE="offline"; echo "WandB disabled"; break;;
        * ) echo "Please answer yes or no.";;
    esac
done

> ".env"
echo "COMPOSE_PROJECT_NAME=wato-asd-training-${USER}" >> ".env"
echo "FIXUID=$FIXUID" >> ".env"
echo "FIXGID=$FIXGID" >> ".env"
echo "USERNAME=${USER}" >> ".env"
echo "WANDB_MODE=${WANDB_MODE}" >> ".env"

if [ $WANDB_MODE == "online" ]; then
    echo "WANDB_KEY=${key}" >> ".env"
fi