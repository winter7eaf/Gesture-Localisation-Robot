#!/usr/bin/python3

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
        if (img is None):
            return None, None
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
    print("Hand Detector Initialized")

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

            length, info, img = detector.findDistance(lm_list[8][0:2], lm_list[12][0:2], img, color=(255, 0, 255), scale=10)

        if img is not None:
            cv2.imshow("Image", img)
        cv2.waitKey(1)


goals = [
    Pose(position=Point(x=2.139872800488411, y=2.4091084324016747, z=0.0),
         orientation=Quaternion(w=0.9999750369921838, x=0.0, y=0.0, z=0.007065790294124714)),
    Pose(position=Point(x=1.7942427343093557, y=10.997075823076422, z=0.0),
         orientation=Quaternion(w=0.6945051411862567, x=0.0, y=0.0, z=0.7194877405945549)),
    Pose(position=Point(x=14.4578006343115, y=10.575440599835618, z=0.0),
         orientation=Quaternion(w=0.9995247774133863, x=0.0, y=0.0, z=0.03082562792094638)),
    Pose(position=Point(x=13.689259857644291, y=21.31942142179531, z=0.0),
         orientation=Quaternion(w=0.6917303017201268, x=0.0, y=0.0, z=0.722155931694937)),
    Pose(position=Point(x=17.286134723080238, y=22.362174253978868, z=0.0),
         orientation=Quaternion(w=0.9942319132574566, x=0.0, y=0.0, z=0.10725158581772617)),
    Pose(position=Point(x=12.420124525361263, y=23.583000051064765, z=0.0),
         orientation=Quaternion(w=0.2607200755719132, x=0.0, y=0.0, z=0.9654144406387217)),
    Pose(position=Point(x=4.485485112377619, y=13.62544721833074, z=0.0),
         orientation=Quaternion(w=0.9426620672762931, x=0.0, y=0.0, z=0.33374874818999023)),
    Pose(position=Point(x=12.377816896015693, y=7.411229829575238, z=0.0),
         orientation=Quaternion(w=0.5067124484646074, x=0.0, y=0.0, z=-0.8621151283738168)),
    Pose(position=Point(x=22.86934957609736, y=8.080257281520984, z=0.0),
         orientation=Quaternion(w=0.9440032004771164, x=0.0, y=0.0, z=-0.3299362930763473)),
]

Table_4 = [goals[0], goals[1], goals[2], goals[7]]
Table_5 = [goals[0], goals[1], goals[2], goals[8]]
Table_3 = [goals[0], goals[1], goals[6]]
Table_2 = [goals[0], goals[1], goals[2], goals[3], goals[5]]
Table_1 = [goals[0], goals[1], goals[2], goals[3], goals[4]]

target_path = None # will be set to one of the above tables


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

will_be_at_home = True

def main():
    global goal_pub, will_be_at_home
    rospy.init_node('hand_track_control', anonymous=True)
    goal_pub = rospy.Publisher('/move_to_goal', Pose, queue_size=10)
    rospy.Subscriber('/move_to_coords', String, from_move_to_coords_callback)

    while True:
        answer = str(start_camera_and_read_hand())
        if answer in ['1', '2', '3', '4', '5']:
            if not will_be_at_home:
                print("Already at table, cannot go to another table")
            print(f"Going to table {answer}")
            will_be_at_home = False
        if answer == '1':
            target_path = Table_1
        elif answer == '2':
            target_path = Table_2
        elif answer == '3':
            target_path = Table_3
        elif answer == '4':
            target_path = Table_4
        elif answer == '5':
            target_path = Table_5
        elif answer == '0':
            if will_be_at_home:
                print("Already at home")
                continue
            target_path = reversed(target_path)
            will_be_at_home = True
        else:
            print("Invalid input")
            continue

        for pose in target_path:
            send_goal(pose)
            while not arrived_yet:
                rospy.sleep(2)
        rospy.sleep(2)

        print(f"Finished path, we are at home = {will_be_at_home}")


if __name__ == "__main__":
    main()
