"""
Hand Tracking Module

Website: https://www.computervision.zone/
"""

import math
import cv2
import mediapipe as mp
import time

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion


class HandDetector:
    """
    Finds Hands using the mediapipe library. Exports the landmarks
    in pixel format. Adds extra functionalities like finding how
    many fingers are up or the distance between two fingers. Also
    provides bounding box info of the hand found.
    """

    def __init__(self, staticMode=False, maxHands=2, modelComplexity=1, detectionCon=0.5, minTrackCon=0.5):

        """
        :param mode: In static mode, detection is done on each image: slower
        :param maxHands: Maximum number of hands to detect
        :param modelComplexity: Complexity of the hand landmark model: 0 or 1.
        :param detectionCon: Minimum Detection Confidence Threshold
        :param minTrackCon: Minimum Tracking Confidence Threshold
        """
        self.staticMode = staticMode
        self.maxHands = maxHands
        self.modelComplexity = modelComplexity
        self.detectionCon = detectionCon
        self.minTrackCon = minTrackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=self.staticMode,
                                        max_num_hands=self.maxHands,
                                        model_complexity=modelComplexity,
                                        min_detection_confidence=self.detectionCon,
                                        min_tracking_confidence=self.minTrackCon)

        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]
        self.fingers = []
        self.lmList = []

    def findHands(self, img, draw=True, flipType=True):
        """
        Finds hands in a BGR image.
        :param img: Image to find the hands in.
        :param draw: Flag to draw the output on the image.
        :return: Image with or without drawings
        """
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        allHands = []
        h, w, c = img.shape
        if self.results.multi_hand_landmarks:
            for handType, handLms in zip(self.results.multi_handedness, self.results.multi_hand_landmarks):
                myHand = {}
                ## lmList
                mylmList = []
                xList = []
                yList = []
                for id, lm in enumerate(handLms.landmark):
                    px, py, pz = int(lm.x * w), int(lm.y * h), int(lm.z * w)
                    mylmList.append([px, py, pz])
                    xList.append(px)
                    yList.append(py)

                ## bbox
                xmin, xmax = min(xList), max(xList)
                ymin, ymax = min(yList), max(yList)
                boxW, boxH = xmax - xmin, ymax - ymin
                bbox = xmin, ymin, boxW, boxH
                cx, cy = bbox[0] + (bbox[2] // 2), \
                         bbox[1] + (bbox[3] // 2)

                myHand["lmList"] = mylmList
                myHand["bbox"] = bbox
                myHand["center"] = (cx, cy)

                if flipType:
                    if handType.classification[0].label == "Right":
                        myHand["type"] = "Left"
                    else:
                        myHand["type"] = "Right"
                else:
                    myHand["type"] = handType.classification[0].label

                wristPoistion = mylmList[0]
                middleMCP = mylmList[9]

                vector = [wristPoistion[0] - middleMCP[0], wristPoistion[1] - middleMCP[1]]
                angle = math.atan2(vector[0], vector[1])

                myHand["angle"] = math.degrees(angle) % 360

                thumbPosition = mylmList[1]
                pinkyPosition = mylmList[17]

                # Determine the hand orientation based on the hand type.
                if myHand["type"] == "Right":
                    if thumbPosition[0] > pinkyPosition[0]:
                        handOrientation = "Front"
                    else:
                        handOrientation = "Back"
                else:
                    if thumbPosition[0] < pinkyPosition[0]:
                        handOrientation = "Front"
                    else:
                        handOrientation = "Back"

                # Flip hand orientation if the angle is between 80 and 260 degrees
                if 80 < myHand["angle"] < 260:
                    if handOrientation == "Front":
                        handOrientation = "Back"
                    elif handOrientation == "Back":
                        handOrientation = "Front"

                myHand["orientation"] = handOrientation

                allHands.append(myHand)

                ## draw
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
                    cv2.rectangle(img, (bbox[0] - 20, bbox[1] - 20),
                                  (bbox[0] + bbox[2] + 20, bbox[1] + bbox[3] + 20),
                                  (255, 0, 255), 2)
                    cv2.putText(img, myHand["type"], (bbox[0] - 30, bbox[1] - 30), cv2.FONT_HERSHEY_PLAIN,
                                2, (255, 0, 255), 2)

        return allHands, img

    def fingersUp(self, myHand):
        """
        Finds how many fingers are open and returns in a list.
        Considers left and right hands separately
        :return: List of which fingers are up
        """
        fingers = []
        myHandType = myHand["type"]
        myLmList = myHand["lmList"]
        myOrientation = myHand["orientation"]
        if self.results.multi_hand_landmarks:
            # Thumb
            if myHandType == "Right":
                fingerUp = myLmList[self.tipIds[0]][0] > myLmList[self.tipIds[0] - 1][0]
            else:
                fingerUp = myLmList[self.tipIds[0]][0] < myLmList[self.tipIds[0] - 1][0]

            if myOrientation == "Back":
                fingerUp = not fingerUp

            if 80 < myHand["angle"] < 260:
                fingerUp = not fingerUp

            fingers.append(1 if fingerUp else 0)

            # 4 Fingers
            for id in range(1, 5):
                if myLmList[self.tipIds[id]][1] < myLmList[self.tipIds[id] - 2][1]:
                    if 80 < myHand["angle"] < 260:
                        fingers.append(0)
                    else:
                        fingers.append(1)
                else:
                    if 80 < myHand["angle"] < 260:
                        fingers.append(1)
                    else:
                        fingers.append(0)
        return fingers

    def findDistance(self, p1, p2, img=None, color=(255, 0, 255), scale=5):
        """
        Find the distance between two landmarks input should be (x1,y1) (x2,y2)
        :param p1: Point1 (x1,y1)
        :param p2: Point2 (x2,y2)
        :param img: Image to draw output on. If no image input output img is None
        :return: Distance between the points
                 Image with output drawn
                 Line information
        """

        x1, y1 = p1
        x2, y2 = p2
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        length = math.hypot(x2 - x1, y2 - y1)
        info = (x1, y1, x2, y2, cx, cy)
        if img is not None:
            cv2.circle(img, (x1, y1), scale, color, cv2.FILLED)
            cv2.circle(img, (x2, y2), scale, color, cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), color, max(1, scale // 3))
            cv2.circle(img, (cx, cy), scale, color, cv2.FILLED)

        return length, info, img


def start_camera_and_read_hand():
    gesture_start_time = None
    last_gesture = None
    capturing = True

    # Initialize the webcam to capture video
    cap = cv2.VideoCapture(0)

    # Initialize the HandDetector class with the given parameters
    detector = HandDetector(staticMode=False, maxHands=1, modelComplexity=1, detectionCon=0.5, minTrackCon=0.5)

    while capturing:
        success, img = cap.read()
        hands, img = detector.findHands(img, draw=True, flipType=True)

        if hands:
            hand = hands[0]
            lm_list = hand["lmList"]

            fingers = detector.fingersUp(hand)
            total_fingers = fingers.count(1)
            stop_camera = False

            if total_fingers == last_gesture:
                if gesture_start_time is None:
                    gesture_start_time = time.time()
                else:
                    countdown = 3 - int(time.time() - gesture_start_time)
                    print(f"Hand = {total_fingers}, countdown: {countdown}", end=" ")
                    cv2.putText(img, str(countdown), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 5)
                    if time.time() - gesture_start_time >= 3:
                        print(f"\nHand 1 Going to: {'table ' + str(total_fingers) if total_fingers != 0 else 'Till'}",
                              end=" ")
                        cv2.putText(img, "table " + str(total_fingers) if total_fingers != 0 else "Till", (45, 375),
                                    cv2.FONT_HERSHEY_PLAIN, 10, (255, 0, 0), 25)

                        gesture_start_time = None

                        confirmation_start_time = time.time()

                        while time.time() - confirmation_start_time < 1:
                            cv2.imshow("Image", img)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break

                            stop_camera = True

            else:
                last_gesture = total_fingers
                gesture_start_time = time.time()

            print(" ")

            if stop_camera:
                cap.release()
                cv2.destroyAllWindows()
                print(f"Final Hand = {total_fingers}")
                return total_fingers

            length, info, img = detector.findDistance(lm_list[8][0:2], lm_list[12][0:2], img, color=(255, 0, 255),
                                                      scale=10)

        cv2.imshow("Image", img)
        cv2.waitKey(1)


goals = [
    Pose(position=Point(x=3.039054575402686, y=3.739731769616226, z=0.0),
         orientation=Quaternion(w=0.8337602955527833, x=0.0, y=0.0, z=0.5521265883470342)),
    Pose(position=Point(x=1.9327985587137941, y=11.423447682767932, z=0.0),
         orientation=Quaternion(w=0.9201964048699748, x=0.0, y=0.0, z=0.39145699184504723)),
    Pose(position=Point(x=15.11636640422266, y=11.425471330763873, z=0.0),
         orientation=Quaternion(w=0.997910297702185, x=0.0, y=0.0, z=0.06461453195633741)),
    Pose(position=Point(x=12.905579134804732, y=7.9848322620675525, z=0.0),
         orientation=Quaternion(w=0.980740448390913, x=0.0, y=0.0, z=-0.19531557257420862)),
    Pose(position=Point(x=9.504558737679664, y=12.853681794445112, z=0.0),
         orientation=Quaternion(w=0.9478868168720372, x=0.0, y=0.0, z=-0.31860725415501284)),
    Pose(position=Point(x=14.295679704866666, y=21.880189496153275, z=0.0),
         orientation=Quaternion(w=0.9999562584662598, x=0.0, y=0.0, z=0.009353136060097923)),
    Pose(position=Point(x=12.955258052470636, y=23.688073509473423, z=0.0),
         orientation=Quaternion(w=0.9775336959333584, x=0.0, y=0.0, z=0.2107792051291313)),
    Pose(position=Point(x=16.279586618366, y=26.657067088902657, z=0.0),
         orientation=Quaternion(w=0.9268607199649388, x=0.0, y=0.0, z=0.37540538859488315)),
    Pose(position=Point(x=16.958741068978618, y=21.78485755495617, z=0.0),
         orientation=Quaternion(w=0.9578953891383724, x=0.0, y=0.0, z=-0.28711743845932836)),
    Pose(position=Point(x=18.948990828185984, y=17.857989217411323, z=0.0),
         orientation=Quaternion(w=0.9674684317201359, x=0.0, y=0.0, z=-0.25299176592328215)),
    Pose(position=Point(x=26.60789029586225, y=17.77152826708097, z=0.0),
         orientation=Quaternion(w=0.9999915213696485, x=0.0, y=0.0, z=0.004117910734322616)),
    Pose(position=Point(x=27.33880758748402, y=27.158517770018815, z=0.0),
         orientation=Quaternion(w=0.9264104237715857, x=0.0, y=0.0, z=-0.376515241029299))
    # You can add more poses here in the same format
]

Table_4 = [goals[1], goals[2], goals[3]]
Table_5 = [goals[1], goals[2], goals[4]]
Table_3 = [goals[1], goals[2], goals[5]]
Table_2 = [goals[1], goals[2], goals[6], goals[7]]
Table_1 = [goals[1], goals[2], goals[6], goals[9]]


def send_goal(pose):
    global arrived_yet
    arrived_yet = False
    goal = Pose()
    goal.position.x = pose.position.x
    goal.position.y = pose.position.y
    goal_pub.publish(goal)
    print("published goal")

arrived_yet = False

def from_move_to_coords_callback(msg):
    global arrived_yet
    if msg.data == "finished":
        arrived_yet = True
        print("arrived")

def main():
    global goal_pub
    rospy.init_node('hand_track_control', anonymous=True)
    goal_pub = rospy.Publisher('/move_to_goal', Pose, queue_size=10)
    rospy.Subscriber('/move_to_coords', String, from_move_to_coords_callback)

    while True:
        answer = start_camera_and_read_hand()
        if answer == 1:
            for pose in Table_1:
                result = send_goal(pose)
                while not arrived_yet:
                    rospy.sleep(1)
            rospy.sleep(1)
        elif answer == 2:
            for pose in Table_2:
                result = send_goal(pose)
                while not arrived_yet:
                    rospy.sleep(1)
            rospy.sleep(1)
        elif answer == 3:
            for pose in Table_3:
                result = send_goal(pose)
                while not arrived_yet:
                    rospy.sleep(1)
            rospy.sleep(1)
        elif answer == 4:
            for pose in Table_4:
                result = send_goal(pose)
                while not arrived_yet:
                    rospy.sleep(1)
            rospy.sleep(1)
        elif answer == 5:
            for pose in Table_5:
                result = send_goal(pose)
                while not arrived_yet:
                    rospy.sleep(1)
            rospy.sleep(1)
        elif answer == 0:
            home = send_goal(goals[0])
            while not arrived_yet:
                rospy.sleep(1)
            rospy.sleep(1)


if __name__ == "__main__":
    main()
