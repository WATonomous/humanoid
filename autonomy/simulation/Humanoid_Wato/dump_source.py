"""
dump_source.py
==============
A diagnostic script that prints the source code definition of the
RetargetingConfig.__post_init__ method from the dex_retargeting library.
This is used to inspect how the library parses configurations internally.
"""
import inspect

from dex_retargeting.retargeting_config import RetargetingConfig

print(inspect.getsource(RetargetingConfig.__post_init__))
