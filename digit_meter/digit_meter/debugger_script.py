import cv2
import mediapipe as mp
import math

def is_finger_raised(finger_name, landmark, idx_dict, threshold=.2):

    base, tip = idx_dict[finger_name]
    tb_x_diff = landmark[base].x-landmark[tip].x
    tb_y_diff = landmark[base].y-landmark[tip].y
    tip_to_base_distance = math.sqrt((tb_x_diff)**2 + (tb_y_diff)**2)

    bw_x_diff = landmark[base].x - landmark[idx_dict['wrist_id']].x
    bw_y_diff = landmark[base].y - landmark[idx_dict['wrist_id']].y
    base_to_wrist_distance = math.sqrt((bw_x_diff)**2 + (bw_y_diff)**2)
    # print("finger: {}, dist_ratio: {}".format(finger_name, tip_to_base_distance/base_to_wrist_distance))
    if tip_to_base_distance/base_to_wrist_distance > .6:
        if finger_name != 'thumb':
            return True
        if finger_name == 'thumb':
        # special case because thumbs are stupid
            
            pinky_tip, pinky_base = idx_dict['pinky']
            tp_x_diff = landmark[tip].x-landmark[pinky_tip].x
            tp_y_diff = landmark[tip].y-landmark[pinky_tip].y
            thumb_tip_to_pinky_tip_distance = math.sqrt((tp_x_diff)**2 + (tp_y_diff)**2)
            pinky_ratio = thumb_tip_to_pinky_tip_distance/base_to_wrist_distance
            index_tip, index_base = idx_dict['index']
            ti_x_diff = landmark[tip].x-landmark[index_tip].x
            ti_y_diff = landmark[tip].y-landmark[index_tip].y
            thumb_tip_to_index_tip_distance = math.sqrt((ti_x_diff)**2 + (ti_y_diff)**2)
            index_ratio = thumb_tip_to_index_tip_distance/base_to_wrist_distance
            print("thumb: pinky_ratio: {}, index_ratio: {}".format(pinky_ratio, index_ratio))
            if (pinky_ratio > 1) and (index_ratio > .5):
                return True

    return False
    




if __name__ == "__main__":
    
    cap = cv2.VideoCapture(0)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)

    mp_hands = mp.solutions.hands
    hand = mp_hands.Hands(max_num_hands=1)
    mp_drawing = mp.solutions.drawing_utils

    finger_landmark_dict = {'index': (5, 8), 'middle': (9, 12), 'ring': (13, 16), 'pinky': (17, 20), 'thumb': (2, 4), 'wrist_id': 0}  #base, tip

    while True:
        success, bgr_frame = cap.read() 
        # convert frame from BGR to RGB

        if success:
            rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            result = hand.process(rgb_frame)
            hand_landmarks = result.multi_hand_landmarks
            # use only first hand
            if not hand_landmarks: # if results.multi_hand_landmarks fails, continue
                continue
            first_hand = hand_landmarks[0]
            num_raised_fingers = 0
            for finger in finger_landmark_dict.keys():
                print("finger", finger)
                if finger == 'wrist_id': continue
                finger_raised = is_finger_raised(finger, first_hand.landmark, finger_landmark_dict)
                # print("finger: {}, dist: {}, raised: {}".format(finger, dist, finger_raised))
                if finger_raised:
                    num_raised_fingers += 1
            print("num_raised_fingers: ", num_raised_fingers)
            cv2.imshow("capture image", rgb_frame)
            if cv2.waitKey(1) == ord('q'): # time in seconds 
                break 
    cap.release()
    cv2.destroyAllWindows()