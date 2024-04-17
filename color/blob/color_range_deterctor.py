
import cv2
import argparse


def callback(value):
    pass


def setup_trackbars(range_filter):
    cv2.namedWindow("Trackbars", 0)

    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255

        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)


def get_trackbar_values(range_filter):
    values = []

    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


def main():
    range_filter = "HSV"

    SOURCE = "video"
    # SOURCE = "image"
    IMAGE_PATH = "D:\Code\Robocon\blob\images\blob1.jpg"  # Replace with the actual path to your image

    setup_trackbars(range_filter)

    if SOURCE == 'video':
        camera = cv2.VideoCapture(0)
        camera.set(3, 320)  # Set resolution
        camera.set(4, 240) # Set resolution
        camera.set(5, 30)
        while True:
            ret, image = camera.read()

            frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)

            thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

            preview = cv2.bitwise_and(image, image, mask=thresh)
            cv2.imshow("camera", image)
            cv2.imshow("Preview", preview)
            cv2.imshow("Thresh", thresh)

            mask = cv2.erode(thresh, None, iterations=2)
            cv2.imshow("errode", mask)

            mask = cv2.dilate(mask,None,iterations=2)
            cv2.imshow("dilate", mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    else:
        # Load the image
        image = cv2.imread(IMAGE_PATH)
        while (True):
            # Convert the image to the desired color space
            frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Get the trackbar values
            v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)

            # Apply thresholding
            thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

            # Apply bitwise AND to get the preview
            preview = cv2.bitwise_and(image, image, mask=thresh)

            # Display the preview and thresholded image
            cv2.imshow("Preview", preview)
            cv2.imshow("Thresh", thresh)

            # Wait for a key press to exit
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()