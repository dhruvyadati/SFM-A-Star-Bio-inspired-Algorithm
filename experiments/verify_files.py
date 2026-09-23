"""Check the saved experiment inventory before running analysis."""
import hashlib
import json
from pathlib import Path


def main():
    base = Path(__file__).resolve().parent
    entries = json.loads((base / 'FILES_SHA256.json').read_text())
    failures = []
    for name, expected in entries.items():
        path = base / name
        if not path.is_file():
            failures.append(name + ': missing')
            continue
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != expected:
            failures.append(name + ': checksum mismatch')
    if failures:
        raise SystemExit('\n'.join(failures))
    print(f'Verified {len(entries)} experiment files.')


if __name__ == '__main__':
    main()
