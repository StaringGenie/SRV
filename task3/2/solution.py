from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

MIN_C = (0, 100, 170)
MAX_C = (2, 255, 255)
HEIGHT = 150



class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def changeLine(self, env, direction):
        rotation_angle = 35
        img, _, _, _ = env.step([1, direction * rotation_angle])
        for i in range(10):
            img, _, _, _ = env.step([1, 0])
            env.render()
        img, _, _, _ = env.step([1, -direction * rotation_angle])

    def canChangeLine(self, env, direction):
        rotation_angle = 35
        img, _, _, _ = env.step([0, direction * rotation_angle])
        env.render()
        contours = self.getContours(env, img)
        img, _, _, _ = env.step([0, -direction * rotation_angle])
        env.render()
        return False if contours else True


    def getContours(self, env, img):
        img = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_BGR2RGB)
        mask = cv2.inRange(img, MIN_C, MAX_C)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0,0])

        
        condition = True
        on_another_line = False
        while condition:
            img, reward, done, info = env.step([1, 0])
            contours = self.getContours(env, img)
            if on_another_line and not contours and self.canChangeLine(env, -1):
                self.changeLine(env, -1)
                on_another_line = False
            if contours:
                _, _, w, h = cv2.boundingRect(contours[0])
                if h > HEIGHT and not on_another_line:
                    self.changeLine(env, 1)
                    on_another_line = True

            env.render()