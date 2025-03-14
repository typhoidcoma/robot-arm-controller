# main.py
import multiprocessing
from simulation_process import simulation_loop
from viewer_process import viewer_loop

if __name__ == "__main__":
    state_queue = multiprocessing.Queue()

    sim_process = multiprocessing.Process(target=simulation_loop, args=(state_queue,))
    vis_process = multiprocessing.Process(target=viewer_loop, args=(state_queue,))

    sim_process.start()
    vis_process.start()

    try:
        sim_process.join()
        vis_process.join()
    except KeyboardInterrupt:
        sim_process.terminate()
        vis_process.terminate()
