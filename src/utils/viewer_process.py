# viewer_process.py
import os
import sys
import time
import mujoco
import mujoco.viewer
import logging
from multiprocessing import Queue

# ----- Configure Logging -----
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)s: %(message)s',
                    datefmt='%H:%M:%S')
logger = logging.getLogger("viewer")

def viewer_loop(state_queue: Queue):
    MODEL_XML = "robots\\trs_so_arm100\\scene.xml"
    if not os.path.exists(MODEL_XML):
        logger.error("Model file '%s' not found!", MODEL_XML)
        sys.exit()
    model = mujoco.MjModel.from_xml_path(MODEL_XML)
    data = mujoco.MjData(model)

    try:
        # Attempt to use the passive viewer if available.
        if hasattr(mujoco.viewer, "launch_passive"):
            logger.info("Launching passive viewer.")
            with mujoco.viewer.launch_passive(model, data) as viewer:
                run_viewer_loop(viewer, model, data, state_queue)
        else:
            logger.info("Passive viewer not available; using standard viewer.")
            with mujoco.viewer.launch(model, data) as viewer:
                run_viewer_loop(viewer, model, data, state_queue)
    except Exception as e:
        logger.error("Error launching viewer: %s", e)
        sys.exit()

def run_viewer_loop(viewer, model, data, state_queue: Queue):
    logger.info("Viewer started. Updating as fast as possible...")
    while viewer.is_running():
        # Drain the queue: apply the latest available state.
        try:
            while True:
                state = state_queue.get_nowait()
                if 'qpos' in state:
                    data.qpos[:] = state['qpos']
                if 'ctrl' in state:
                    data.ctrl[:] = state['ctrl']
                # After updating state from the queue, run forward pass to update geometry.
                mujoco.mj_forward(model, data)
        except Exception:
            # Queue is empty
            pass

        try:
            viewer.sync()
        except Exception as e:
            logger.error("Viewer sync error: %s", e)
        # Minimal sleep to yield CPU, adjust or remove as needed.
        time.sleep(0.001)

if __name__ == "__main__":
    from multiprocessing import Queue
    state_queue = Queue()
    viewer_loop(state_queue)
