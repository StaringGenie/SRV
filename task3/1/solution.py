from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

MIN_C = (0, 100, 170)
MAX_C = (2, 255, 255)
HEIGHT = 150



class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0,0])

        
        condition = True
        while condition:
            img, reward, done, info = env.step([1, 0])
            img = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_BGR2RGB)

            mask = cv2.inRange(img, MIN_C, MAX_C)
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                _, _, _, h = cv2.boundingRect(contours[0])
                if h > HEIGHT:
                    condition = False
                    img, _, _, _ = env.step([0,0])
            env.render()