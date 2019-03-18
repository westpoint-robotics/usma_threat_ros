# ------------------------- BODY, FACE AND HAND MODELS -------------------------
# Downloading body pose (COCO and MPI), face and hand models
OPENPOSE_URL="http://posefs1.perception.cs.cmu.edu/OpenPose/models/"
POSE_FOLDER="cfg/"

# ------------------------- POSE MODELS -------------------------
# Body (COCO)
COCO_FOLDER=${POSE_FOLDER}"coco/"
mkdir ${COCO_FOLDER}
echo "${OPENPOSE_URL}pose/coco/pose_iter_440000.caffemodel"
wget -c "${OPENPOSE_URL}pose/coco/pose_iter_440000.caffemodel" -P ${COCO_FOLDER}

# Body (MPI)
MPI_FOLDER=${POSE_FOLDER}"mpi/"
mkdir ${MPI_FOLDER}
wget -c "${OPENPOSE_URL}pose/mpi/pose_iter_160000.caffemodel" -P ${MPI_FOLDER}

# "------------------------- FACE MODELS -------------------------"
# Face
FACE_FOLDER=${POSE_FOLDER}"face/"
mkdir ${FACE_FOLDER}
wget -c "${OPENPOSE_URL}face/pose_iter_116000.caffemodel" -P ${FACE_FOLDER}

# "------------------------- HAND MODELS -------------------------"
# Hand
HAND_FOLDER=${POSE_FOLDER}"hand/"
mkdir ${HAND_FOLDER}

wget -c "${OPENPOSE_URL}hand/pose_iter_102000.caffemodel" -P ${HAND_FOLDER}
