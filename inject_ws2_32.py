import platform

Import("env")


def _is_native_env(environment):
    return str(environment.get("PIOPLATFORM", "")).lower() == "native"


if platform.system() == "Windows" and _is_native_env(env):
    env.Append(LIBS=["ws2_32"])
    print("[Astra-Rocket Native] Injected Windows native link lib: ws2_32")
