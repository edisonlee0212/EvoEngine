"""Execute the source notebook into a derived output, preserving source and historical results."""
import argparse
from pathlib import Path

import nbformat
from jupyter_client import KernelManager


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    notebook = nbformat.read(root / "Scripts/soybean_experiment_analysis.ipynb", as_version=4)
    # Use the installed scientific kernel through Jupyter's public client API; no extra packages are installed.
    manager = KernelManager(kernel_name="python3")
    manager.start_kernel(cwd=str(root))
    client = manager.blocking_client()
    client.start_channels()
    try:
        client.wait_for_ready(timeout=120)
        info_id = client.kernel_info()
        while True:
            info = client.get_shell_msg(timeout=30)
            if info.get("parent_header", {}).get("msg_id") == info_id:
                notebook.metadata.language_info = info["content"]["language_info"]
                break
        for cell in notebook.cells:
            if cell.cell_type != "code":
                continue
            cell.outputs = []
            message_id = client.execute(cell.source, stop_on_error=True)
            while True:
                message = client.get_iopub_msg(timeout=1200)
                if message.get("parent_header", {}).get("msg_id") != message_id:
                    continue
                kind, content = message["header"]["msg_type"], message["content"]
                if kind == "execute_input":
                    cell.execution_count = content["execution_count"]
                elif kind in {"stream", "display_data", "execute_result", "error"}:
                    cell.outputs.append(nbformat.v4.output_from_msg(message))
                if kind == "error":
                    raise RuntimeError(f"Notebook cell failed: {content['ename']}: {content['evalue']}")
                if kind == "status" and content["execution_state"] == "idle":
                    break
    finally:
        client.stop_channels()
        manager.shutdown_kernel()
        nbformat.write(notebook, args.output or args.study / "analysis/study_analysis.executed.ipynb")
