import math, pickle

with open("right_dict.pickle", "rb") as file1:
    right_dict = pickle.load(file1)


with open("left_dict.pickle", "rb") as file1:
    left_dict = pickle.load(file1)


def steps_to_sequences(steps):
    # U R2 U B D2 L F U2 R2 D F' R F2 U' L2 D' R2 L2 D R2 U L2
    steps_list  = steps.replace("U", "W").replace("D", "Y").replace("L", "O").replace("R", "R").replace("F", "G").replace("B", "B").split(" ")
    steps_list = [i for i in steps_list if i]
    action_list = []
    for i in steps_list:
        if len(i) == 1:
            action_list.append((i, math.pi / 2, True))
        elif "2" in i:
            action_list.append((i[:-1], math.pi, True))
        elif "'" in i:
            action_list.append((i[:-1], math.pi / 2, False))
    
    color_space_dict = {
        "W": "U",
        "Y": "D",
        "O": "L",
        "R": "R",
        "G": "F",
        "B": "B"
    }

    hand_available_space = {
        "left": ["L", "U"],
        "right": ["R", "U"]
    }

    holded_hand = "right"
    free_hand = "left"

    left_conf_list = []
    left_jawwidth_list = []
    left_pose_list = []

    right_conf_list = []
    right_jawwidth_list = []
    right_pose_list = []

    conf_list = []


    for i in action_list:
        color_face = i[0]
        rotation_angle = i[1]
        rotation_direction = i[2]
        # conf_list.append(f"面: {color_face}, 角度: {rotation_angle}, 方向: {rotation_direction}")
        if color_space_dict.get(color_face) in hand_available_space.get(free_hand):
            if color_space_dict.get(color_face) == "U":
                conf_list.append(f"{free_hand}_wait2tmove")
            else:
                conf_list.append(f"{free_hand}_wait2move")
            conf_list.append(f"{free_hand}_spin_{rotation_angle}_{rotation_direction}")

            # 需要修改末端轴的角度
            # conf_list.append(f"需要修改末端轴的角度")

            if color_space_dict.get(color_face) == "U":
                conf_list.append(f"{free_hand}_tmove2wait_spanback")
            else:
                conf_list.append(f"{free_hand}_move2wait_spanback")
        elif color_space_dict.get(color_face) in hand_available_space.get(holded_hand):
            conf_list.append(f"{free_hand}_wait2hold")
            conf_list.append(f"{holded_hand}_hold2wait")

            holded_hand, free_hand = free_hand, holded_hand

            conf_list.append(f"{free_hand}_wait2move")
            conf_list.append(f"{free_hand}_spin_{rotation_angle}_{rotation_direction}")
            # 需要修改末端轴的角度
            # conf_list.append(f"需要修改末端轴的角度")
            conf_list.append(f"{free_hand}_move2wait_spanback")

            


        if color_space_dict.get(color_face) in ["D", "F", "B"]:
            cube_rotation_angle = math.pi / 2
            if color_space_dict.get(color_face) == "F":
                if holded_hand == "left":
                    cube_rotation_direction = False
                else:
                    cube_rotation_direction = True
            elif color_space_dict.get(color_face) == "B":
                if holded_hand == "left":
                    cube_rotation_direction = True
                else:
                    cube_rotation_direction = False
            elif color_space_dict.get(color_face) == "D":
                cube_rotation_direction = True
                cube_rotation_angle = math.pi
            conf_list.append(f"{holded_hand}_spin_{cube_rotation_angle}_{cube_rotation_direction}")
            color_space_dict[color_face] = "U"
            temp_y = "WBYGWBYG"
            for i in range(len(temp_y)):
                if temp_y[i] == color_face:
                    color_space_dict[temp_y[i+1]] = "B"
                    color_space_dict[temp_y[i+2]] = "D"
                    color_space_dict[temp_y[i+3]] = "F"
                    break


            conf_list.append(f"{free_hand}_wait2hold")
            # 需要修改末端轴的角度
            # conf_list.append(f"需要修改末端轴的角度")
            conf_list.append(f"{holded_hand}_hold2wait_spanback")
            holded_hand, free_hand = free_hand, holded_hand
            conf_list.append(f"{free_hand}_wait2tmove")
            conf_list.append(f"{free_hand}_spin_{rotation_angle}_{rotation_direction}")
            # conf_list.append(f"需要修改末端轴的角度")
            conf_list.append(f"{free_hand}_tmove2wait_spanback")

    print(action_list)
    print(conf_list)
    for i in conf_list:
        print(i)
    return(conf_list)


steps_to_sequences("L' D' R U'")

dual_dcit = dict(left_dict, **right_dict)
dual_dcit["left_wait2tmove"] = [dual_dcit["left_wait2twait"][0] + dual_dcit["left_twait2tmove"][0], dual_dcit["left_wait2twait"][1] + dual_dcit["left_twait2tmove"][1], dual_dcit["left_wait2twait"][2] + dual_dcit["left_twait2tmove"][2]]
dual_dcit["left_tmove2wait"] = [dual_dcit["left_tmove2twait"][0] + dual_dcit["left_twait2wait"][0], dual_dcit["left_tmove2twait"][1] + dual_dcit["left_twait2wait"][1], dual_dcit["left_tmove2twait"][2] + dual_dcit["left_twait2wait"][2]]
dual_dcit["right_wait2tmove"] = [dual_dcit["right_wait2twait"][0] + dual_dcit["right_twait2tmove"][0], dual_dcit["right_wait2twait"][1] + dual_dcit["right_twait2tmove"][1], dual_dcit["right_wait2twait"][2] + dual_dcit["right_twait2tmove"][2]]
dual_dcit["right_tmove2wait"] = [dual_dcit["right_tmove2twait"][0] + dual_dcit["right_twait2wait"][0], dual_dcit["right_tmove2twait"][1] + dual_dcit["right_twait2wait"][1], dual_dcit["right_tmove2twait"][2] + dual_dcit["right_twait2wait"][2]]

with open("dual_dict.pickle", "wb") as file1:
    pickle.dump(dual_dcit, file1)