file1 = open('D:/agen3/dataset/putty.log', 'r')
Lines = file1.readlines()

state = 0



poses = []

last_pos = []

for line in Lines:
    if "goals : " in line and state == 0:
        state = 1
        last_pos = []
    elif "[(" in line and state == 1:
        state = 2
        last_pos.append(line)
    elif "[(" in line and state == 2:
        state = 3
        last_pos.append(line)
    elif "[" in line and state == 3:
        state = 4
        last_pos.append(line)
    elif "pass" in line and state == 4:
        state = 0
        poses.append(last_pos)
        
    else:
        state = 0
        

print(poses)
print(len(poses))