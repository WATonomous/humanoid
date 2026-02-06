Import("env")

env_name = env["PIOENV"]

env.Replace(PROGNAME=env_name)
