import psutil
import subprocess
from typing import Optional


def get_cpu_usage_percent() -> float:
    return psutil.cpu_percent(interval=None)


def get_ram_usage_percent() -> float:
    return psutil.virtual_memory().percent


def get_cpu_temp_c() -> Optional[float]:
    try:
        output = subprocess.check_output(["vcgencmd", "measure_temp"], text=True).strip()
        return float(output.split("=")[1].split("'")[0])
    except Exception:
        return None
