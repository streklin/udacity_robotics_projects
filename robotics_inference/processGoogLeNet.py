file_location = 'results_googlenet.txt'

text_file = open(file_location, "r")
lines = text_file.readlines()

living_room = []
for i in range(74):
    living_room.append(i)



dining_room = []
for i in range(74, 113):
    dining_room.append(i)

for i in range(183, 205):
    dining_room.append(i)

kitchen = []
for i in range(113, 184):
    kitchen.append(i)

hallway = []
for i in range(205, 339):
    hallway.append(i)

bathroom = []
for i in range(339, 355):
    bathroom.append(i)

fail_count = 0

for l in lines:
    if l.startswith("Image:"):
        tokens = l.split(' ')
        img_parts = tokens[1].split('_')
        img_num = int(img_parts[1].split('.')[0])

        classification = tokens[3].replace(" ", "").strip()

        prediction = ""
        if img_num in living_room:
            prediction = "LIVING"

        if img_num in dining_room:
            prediction = "DINING"

        if img_num in kitchen:
            prediction = "KITCHEN"

        if img_num in hallway:
            prediction = "HALLWAY"

        if img_num in bathroom:
            prediction = "BATHROOM"

        if classification != prediction:
            print(img_num, classification, prediction)
            fail_count = fail_count + 1

print("FAIL COUNT: ", fail_count)

text_file.close()