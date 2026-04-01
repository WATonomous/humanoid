import inspect
from dex_retargeting.retargeting_config import RetargetingConfig

sig = inspect.signature(RetargetingConfig.__init__)
print("--- RetargetingConfig.__init__ parameters ---")
for name, param in sig.parameters.items():
    print(f"{name}: {param.annotation}")
