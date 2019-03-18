import random
bboxes = {"person":[], "backpack":[]}
bboxes["person"].append([random.randint(1,101), random.randint(1,101), random.randint(1,101), random.randint(1,101)])
bboxes["person"].append([random.randint(1,101), random.randint(1,101), random.randint(1,101), random.randint(1,101)])
bboxes["person"].append([random.randint(1,101), random.randint(1,101), random.randint(1,101), random.randint(1,101)])
bboxes["person"].append([random.randint(1,101), random.randint(1,101), random.randint(1,101), random.randint(1,101)])


for person_box in bboxes["person"]:
    print("person corners: {}").format(person_box[0]) 





import random
gun_boxes = []
human_boxes = []


human_boxes.append({'xmin':random.randint(1,101),'ymin':random.randint(1,101),'xmax':random.randint(1,101),'ymax':random.randint(1,101)})
human_boxes.append({'xmin':random.randint(1,101),'ymin':random.randint(1,101),'xmax':random.randint(1,101),'ymax':random.randint(1,101)})
human_boxes.append({'xmin':random.randint(1,101),'ymin':random.randint(1,101),'xmax':random.randint(1,101),'ymax':random.randint(1,101)})
human_boxes.append({'xmin':random.randint(1,101),'ymin':random.randint(1,101),'xmax':random.randint(1,101),'ymax':random.randint(1,101)})


