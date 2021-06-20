import cv2
import os

image_folder = 'frames'
video_name = 'video.avi'

print(os.listdir(image_folder))
images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
images = sorted(images, key=lambda x: int(x.split('.')[0].split('e')[1]))
print(images)
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(video_name, 0, 10, (width,height))

for i in range(len(images)):
    if i%10==0:
        image = images[i]
        video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()