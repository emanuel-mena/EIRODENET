Import("env")

from pathlib import Path


"""Genera durante el build un header C ignorado por Git a partir de .env."""

def parse_env(path):
    """Lee asignaciones KEY=VALUE simples sin revelar sus valores en consola."""
    values = {}
    if path.exists():
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def c_string(value):
    """Escapa un valor para insertarlo de forma segura en un literal C."""
    return value.replace("\\", "\\\\").replace('"', '\\"')


project_dir = Path(env.subst("$PROJECT_DIR"))
output = project_dir / "include" / "generated_secrets.h"
values = parse_env(project_dir / ".env")
content = (
    "#pragma once\n"
    f'#define PROJECT_WIFI_SSID "{c_string(values.get("WIFI_SSID", values.get("WIFI_SSD", "")))}"\n'
    f'#define PROJECT_WIFI_PASSWORD "{c_string(values.get("WIFI_PASS", ""))}"\n'
)
if not output.exists() or output.read_text(encoding="utf-8") != content:
    output.write_text(content, encoding="utf-8")
