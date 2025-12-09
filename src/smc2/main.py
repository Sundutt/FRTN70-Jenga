# main.py
import threading
import queue
import time
from video_thread import VideoWorker
from robot_thread import RobotWorker

def main():
    # Command queue: video -> robot
    cmd_queue = queue.Queue()
    # Response queue: robot -> video
    resp_queue = queue.Queue()

    # Create and start robot worker thread
    robot = RobotWorker(cmd_queue, resp_queue)
    robot_thread = threading.Thread(target=robot.run, name="RobotThread", daemon=True)
    robot_thread.start()

    # Create and start video worker (runs in main thread by default or new thread)
    video = VideoWorker(cmd_queue, resp_queue)
    video_thread = threading.Thread(target=video.run, name="VideoThread")
    video_thread.start()

    try:
        while video_thread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt — shutting down...")
        video.stop()
        cmd_queue.put(("shutdown", None))
        robot_thread.join(timeout=2)
        video_thread.join(timeout=2)

if __name__ == "__main__":
    main()