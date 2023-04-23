import cv2
import numpy as np
from rrt import rrt, rrt_star, rrt_star_2

k = 0
last_time_obstacles = None
path = []
discard = []


def create_map(boxes):
    global k
    global path
    global discard
    img_result = np.ones((800, 800, 3), dtype=np.uint8)
    #Круг-робот
    #robot_coordinates = (400, 450)
    robot_coordinates = (390, 590)
    robot_color = (0, 255, 0)
    robot_radius = 3
    robot_thickness = -1
    #Круг-цель
    goal_radius = robot_radius
    goal_thickness = robot_thickness
    goal_color = (0, 165, 255)

    cv2.circle(img_result, (int(robot_coordinates[0]), int(robot_coordinates[1])),
               robot_radius, robot_color, robot_thickness)
    obstacles = []  # Fill this with the coordinates of the red squares

    for box in boxes:
        if len(box) > 0:
            print("box[0]", box[0])
            _, l_d, r_d, _ = box[0]
            print(l_d)
            # r_u = np.array(r_d)
            M = np.array([[-1.96945371e-01, -1.28523387e+00, 4.21924893e+02],
                          [-8.31273622e-02, -1.87058363e+00, 6.16781325e+02],
                          [-1.82760476e-04, -3.31891257e-03, 1.00000000e+00]])

            pts_src = np.array([(l_d[0][0], l_d[0][1] + 5), (r_d[0][0], r_d[0][1] + 5), r_d[0], l_d[0]])

            # Создаем массив для точек во втором изображении и преобразуем точки из первого изображения
            pts_dst = cv2.perspectiveTransform(np.array([pts_src], dtype=np.float32), M)[0]

            # Convert floating-point values to integers
            pts_dst = pts_dst.astype(int)
            print("pts_dst", pts_dst)
            obs = (pts_dst[0].tolist(), pts_dst[2].tolist())
            obstacles.append(obs)
            pt1 = tuple(pts_dst[0].tolist())
            pt2 = tuple(pts_dst[2].tolist())

            # Рисуем прямоугольник на втором изображении
            # img_result = np.ones((800, 800, 3), dtype=np.uint8)
            cv2.rectangle(img_result, pt1, pt2, (0, 0, 255), thickness=-1)

    ###
    start = robot_coordinates
    goal = (600, 200)
    cv2.circle(img_result, (int(goal[0]), int(goal[1])), goal_radius, goal_color,
               goal_thickness)
    bounds = ((0, 0), (799, 799))
    if len(obstacles) > 0:
        print("Obstacles:\n", obstacles)

    #Проверка на изменения положений препятствий
    changes = check(obstacles)
    if (k == 0) or changes:
        # path = rrt(start, goal, bounds, obstacles)

        path, discard = rrt_star(start, goal, bounds, obstacles, step_size=40)

    #path, discard = rrt_star_2(start, goal, bounds, obstacles)
    print("Discard:", discard)
    if path is not None:
        print("Found path from", start, "to", goal, ":")
        print(path)

        for j in range(len(discard) - 1):
            dis_pt_1 = (int(discard[j][0][0]), int(discard[j][0][1]))
            dis_pt_2 = (int(discard[j][1][0]), int(discard[j][1][1]))
            cv2.line(img_result, dis_pt_1, dis_pt_2, (128, 128, 128), thickness=1)
        for i in range(len(path) - 1):
            pt1 = (int(path[i][0]), int(path[i][1]))
            pt2 = (int(path[i + 1][0]), int(path[i + 1][1]))
            cv2.line(img_result, pt1, pt2, (255, 255, 0), thickness=1)

        ##########Перевод пути в метры
        metr_in_pixel = 1.5 / 100
        metr_path = [(node_x * metr_in_pixel, node_y * metr_in_pixel) for node_x, node_y in path]
        print("Path in meters:", metr_path)
        ###################################################


        #########Создание пути для настоящего робота в метрах, где (0,0) - его начальное положение
        # first_x, first_y = path[0]
        first_x, first_y = robot_coordinates
        path_real_life = [(x - first_x, -y + first_y) for x, y in path]
        metr_path_real_life = [(node_x * metr_in_pixel, node_y * metr_in_pixel) for node_x, node_y in path_real_life]
        # print("Путь в реале в пкселях", path_real_life)
        print("Маршрут для реального робота в метрах:", metr_path_real_life)
        ###########
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.25
        thickness = 1
        for i in range(len(path)):
            cv2.putText(img_result, f"{(round(metr_path_real_life[i][0], 2), round(metr_path_real_life[i][1], 2))}",
                        (int(path[i][0]), int(path[i][1])), font,
                        font_scale, (255,255,255), thickness, cv2.LINE_AA)

    else:
        print("Failed to find path from", start, "to", goal)
    ###

    # global k
    cv2.imwrite(fr'C:\Users\Eugene\PycharmProjects\vichFon\maps\map{k}.png', img_result)
    k += 1

    return metr_path_real_life, obstacles


def check(my_list):
    # Сохраняем предыдущее значение списка
    global last_time_obstacles
    if last_time_obstacles is None:
        last_time_obstacles = my_list.copy()

    # Проверяем, изменился ли список
    if my_list != last_time_obstacles:
        print("Список изменился")
        return True
    else:
        print("Список не изменился")
        return False

    # Обновляем значение prev_list
    last_time_obstacles = my_list.copy()




