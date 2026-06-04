"""
dump_kwargs.py
==============
A small introspection script that inspects and prints the parameters and arguments
accepted by the RetargetingConfig constructor from the dex_retargeting library.
This is used during development to make sure our dynamically constructed config dictionaries
match the constructor parameters of dex_retargeting.
"""
import inspect

from dex_retargeting.retargeting_config import RetargetingConfig

sig = inspect.signature(RetargetingConfig.__init__)
print("--- RetargetingConfig.__init__ parameters ---")
for name, param in sig.parameters.items():
    print(f"{name}: {param.annotation}")
