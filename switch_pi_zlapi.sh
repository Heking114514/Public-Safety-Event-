#!/usr/bin/env bash
set -euo pipefail

AGENT_DIR="${PI_AGENT_DIR:-$HOME/.pi/agent}"
MODELS_FILE="$AGENT_DIR/models.json"
AUTH_FILE="$AGENT_DIR/auth.json"
PROVIDER_ID="${PI_PROVIDER_ID:-zlapi}"

usage() {
    printf 'Usage: %s [base_url] [api_key]\n' "$0"
    printf '\n'
    printf 'Without arguments, the script prompts for both values.\n'
    printf 'The URL should be the provider API base URL, for example: https://host.example/v1\n'
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

if [[ $# -gt 2 ]]; then
    usage >&2
    exit 2
fi

if [[ -n "${1:-}" ]]; then
    base_url="$1"
else
    read -r -p "New base URL: " base_url
fi

base_url="${base_url%/}"
if [[ -z "$base_url" ]]; then
    printf 'Error: base URL cannot be empty.\n' >&2
    exit 1
fi

if [[ -n "${2:-}" ]]; then
    api_key="$2"
else
    read -r -s -p "New API key: " api_key
    printf '\n'
fi

if [[ -z "$api_key" ]]; then
    printf 'Error: API key cannot be empty.\n' >&2
    exit 1
fi

PI_SWITCH_AGENT_DIR="$AGENT_DIR" \
PI_SWITCH_PROVIDER_ID="$PROVIDER_ID" \
PI_SWITCH_BASE_URL="$base_url" \
PI_SWITCH_API_KEY="$api_key" \
python3 - <<'PY'
import json
import os
import pathlib
import tempfile

agent_dir = pathlib.Path(os.environ["PI_SWITCH_AGENT_DIR"])
provider_id = os.environ["PI_SWITCH_PROVIDER_ID"]
base_url = os.environ["PI_SWITCH_BASE_URL"]
api_key = os.environ["PI_SWITCH_API_KEY"]
models_path = agent_dir / "models.json"
auth_path = agent_dir / "auth.json"


def load(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise SystemExit(f"Missing configuration file: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Invalid JSON in {path}: {exc}") from exc


def write_json(path, data, mode):
    text = json.dumps(data, ensure_ascii=False, indent=2) + "\n"
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temp_path = pathlib.Path(temp_name)
    try:
        os.fchmod(fd, mode)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_path, path)
    finally:
        temp_path.unlink(missing_ok=True)


models = load(models_path)
auth = load(auth_path)

providers = models.get("providers")
if not isinstance(providers, dict) or provider_id not in providers:
    raise SystemExit(f"Provider '{provider_id}' was not found in {models_path}")
if not isinstance(auth.get(provider_id), dict):
    raise SystemExit(f"Provider '{provider_id}' was not found in {auth_path}")

providers[provider_id]["baseUrl"] = base_url
auth[provider_id]["key"] = api_key

write_json(models_path, models, 0o600)
write_json(auth_path, auth, 0o600)
print(f"Updated {provider_id} URL and API key.")
print(f"Preserved all other settings in {models_path} and {auth_path}.")
PY
