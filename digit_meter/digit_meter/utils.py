import math

def is_finger_raised(finger_name, landmark, idx_dict):

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
            # print("thumb: pinky_ratio: {}, index_ratio: {}".format(pinky_ratio, index_ratio))
            if (pinky_ratio > 1) and (index_ratio > .5):
                return True

    return False