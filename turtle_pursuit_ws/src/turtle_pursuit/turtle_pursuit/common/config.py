from pathlib import Path
import yaml

def load_config(path):
    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or 'control' not in data or 'match' not in data:
        raise ValueError('configuration requires control and match sections')
    return data

