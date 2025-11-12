#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
from sklearn.cluster import KMeans
from ultralytics import YOLO
import json

hex_colors = [
    "#1f77b4",  # blue
    "#ff7f0e",  # orange
    "#2ca02c",  # green
    "#d62728",  # red
    "#9467bd",  # purple
    "#8c564b",  # brown
    "#e377c2",  # pink
    "#7f7f7f",  # gray
    "#bcbd22",  # olive
    "#17becf"   # cyan
]

# Convert hex to RGB tuples
COLOURS = [
    tuple(int(c.lstrip('#')[i:i+2], 16) for i in (0, 2, 4))
    for c in hex_colors
]


class ImageSyncProcessor(Node):
    def __init__(self):
        super().__init__('image_sync_processor')

        self.bridge = CvBridge()

        # Load YOLO model (will auto-download if not found)
        # self.yolo_model = YOLO('yolo11s.pt')
        # self.yolo_model = YOLO('yolo11n.pt')
        # self.yolo_model = YOLO('hand_raise.pt')
        self.yolo_model = YOLO('yollo11s_leaf_v4.pt')

        # Interpolation parameters: [slope, intercept] for pixel shift calculation
        # interp_params[0] for distance < 30, interp_params[1] for distance >= 30
        self.interp_params = [
            [814, 5.89],  # [slope, intercept] for distance < 30
            [1431, -6.43]   # [slope, intercept] for distance >= 30
        ]

        # Create subscribers for compressed images
        self.webcam_sub = message_filters.Subscriber(
            self,
            CompressedImage,
            '/webcam/image/compressed'
        )
        self.tof_sub = message_filters.Subscriber(
            self,
            CompressedImage,
            '/tof/image/compressed'
        )

        # Create approximate time synchronizer
        # slop parameter defines the maximum time difference (in seconds) between messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.webcam_sub, self.tof_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance for synchronization
        )
        self.ts.registerCallback(self.sync_callback)

        # Publishers for the processed overlay image
        self.overlay_pub = self.create_publisher(
            Image,
            '/processed/overlay_image',
            10
        )
        self.overlay_compressed_pub = self.create_publisher(
            CompressedImage,
            '/processed/overlay_image/compressed',
            10
        )

        self.overlay_combined_pub = self.create_publisher(
            Image,
            '/processed/overlay_combined_image',
            10
        )
        self.overlay_combined_compressed_pub = self.create_publisher(
            CompressedImage,
            '/processed/overlay_combined_image/compressed',
            10
        )

        # Publisher for detection labels and distances
        self.detections_pub = self.create_publisher(
            String,
            '/detections/labels_distances',
            10
        )

        self.get_logger().info('Image Sync Processor node started')
        self.get_logger().info('Subscribing to /webcam/image/compressed and /tof/image/compressed')
        self.get_logger().info('Publishing overlay to /processed/overlay_image and /processed/overlay_image/compressed')
        self.get_logger().info('Publishing detections to /detections/labels_distances')


    def do_kmeans(self, filtered_image, n_samples = 5):
        pixels = filtered_image.reshape(-1, 1)

        np.random.seed(42)

        # Define the size of the random sample
        sample_size = 3000

        # Take a random sample from the pixels array
        random_sample = np.random.choice(pixels.flatten(), size=sample_size)
        random_sample=random_sample.reshape(-1,1)


        # Perform k-means clustering
        k = n_samples  # Number of clusters
        kmeans = KMeans(n_clusters=k, random_state=0,n_init=10)
        kmeans.fit(random_sample)#pixels)
        
        cluster_centers = kmeans.cluster_centers_
        clusters = kmeans.predict(pixels)
        

        clustered_image = cluster_centers[clusters].reshape(filtered_image.shape)
        # Convert the spread image to 8-bit for visualization
        cluster_centers_flat = np.sort(np.uint8(cluster_centers.flatten()))
        # self.get_logger().info(f'Cluster centers (sorted): {cluster_centers_flat}')
        clustered_image = np.uint8(clustered_image)
        dists = ((255-np.float32(cluster_centers_flat))*200/255)
        dists = np.where(dists>1,dists,1)
        return clustered_image, cluster_centers, dists
    
    def draw_instance_segmentation_masks(self, img, masks):
        ''' Draws coloured polygons masks over img '''
        filled = np.zeros_like(img)
        # image_ret = img.copy()

        for i, mask in enumerate(masks):
            if True or i==(len(masks) -1):
                filled[mask] = COLOURS[i % len(COLOURS)]
        # Blend original and filled into a composite image
        cv2.addWeighted(img, 0.75, filled, 0.45, 0.0, dst=img)
    
    def draw_instance_segmentation_masks_gl(self, img, masks,dists):
        ''' Draws gray level polygons masks over img '''
        if img.ndim == 3 and img.shape[2] == 3:  # RGB image
            filled = np.zeros(img.shape[:2], dtype=img.dtype)  # grayscale version
        else:
            filled = np.zeros_like(img)  # already grayscale or single channel
        image_ret = img.copy()

        for i, mask in enumerate(masks):
            if True or i==(len(masks) -1):
                filled[mask] = dists[i] #COLOURS[i % len(COLOURS)]
        # Blend original and filled into a composite image
        cv2.addWeighted(image_ret, 0.75, filled, 0.45, 0.0, dst=img)
        return image_ret

    def get_segmentation(self, clustered_image, cluster_centers,min_region_size=1000):
        num_grps = len(cluster_centers)
        # Flatten cluster_centers if it's 2D
        centers_flat = cluster_centers.flatten() if cluster_centers.ndim > 1 else cluster_centers
        gray_level_ranges = [(float(c)-1, float(c)+1) for c in centers_flat]
        
        segmentation_masks = [] #np.zeros_like(clustered_image, dtype=np.uint8)
        stats = []
        lbls = []
        for i, (min_level, max_level) in enumerate(gray_level_ranges):
            segmentation_mask = np.zeros_like(clustered_image, dtype=bool)#np.uint8)
            # Apply thresholding to create a binary image for the current region
            _, binary_image = cv2.threshold(clustered_image, min_level, 255, cv2.THRESH_BINARY)
            _, upper_thresholded = cv2.threshold(clustered_image, max_level, 255, cv2.THRESH_BINARY_INV)
            binary_image = cv2.bitwise_and(binary_image, upper_thresholded)

            # Apply connected component labeling
            
            num_labels_, labels_, stats_, _ = cv2.connectedComponentsWithStats(binary_image, connectivity=8)
            

            # Assign a unique label to the corresponding pixels in the segmentation masks
            p = 1
            for p, st in enumerate(stats_):# range(q, q+num_labels_-1):
                if p==0:
                    continue        
                elif st[cv2.CC_STAT_AREA] < min_region_size:
                    segmentation_mask[labels_ == p] = 0
                    
                else:
                    stats.append(st)
                    segmentation_mask[labels_ == p] =True #centers_flat[i]
                    segmentation_masks.append(segmentation_mask)
                    lbls.append(centers_flat[i]) #(255.0-centers_flat[i])*400.0/255.0)

        sorted_index = sorted(range(len(lbls)), key=lambda k: lbls[k])

        # Sort l1
        lbls = sorted(lbls)

        # Reorder l2 and l3 using the sorted index
        stats = [stats[i] for i in sorted_index]
        segmentation_masks = [segmentation_masks[i] for i in sorted_index]    
        return lbls,stats,segmentation_masks
    
    def get_shifted_image(self, clustered_image, cluster_centers): 
        shifted_image = np.zeros_like(clustered_image, dtype=np.float32)

        centers_flat = cluster_centers.flatten() if cluster_centers.ndim > 1 else cluster_centers
        gray_level_ranges = [(float(c)-1, float(c)+1) for c in centers_flat]
        yn, xn = clustered_image.shape

        for i, (min_level, max_level) in enumerate(gray_level_ranges):
            # Create mask for current cluster
            mask = cv2.inRange(clustered_image, min_level, max_level)
            if np.count_nonzero(mask) == 0:
                continue

            # Compute pixel shift based on distance
            distance = (255.001 - centers_flat[i]) * 200.0 / 255.0
            if distance < 40.0: #38.0:
                slope, intercept = self.interp_params[0]
            else:
                slope, intercept = self.interp_params[1]
            pixel_shift = slope * (1.0 / distance) + intercept
            pixel_shift = int(round(pixel_shift))

            # Apply negative shift to the clustered image for this cluster
            temp = np.zeros_like(shifted_image)
            temp[mask > 0] = centers_flat[i]
            left_side = np.zeros((yn, pixel_shift), dtype=np.float32)
            temp = np.concatenate((left_side, temp), axis=1)
            temp = temp[:, :xn]
            # y_coords, x_coords = np.where(mask > 0)

            # for y, x in zip(y_coords, x_coords):
            #     source_x = x - pixel_shift  # negative shift
            #     if 0 <= source_x < clustered_image.shape[1]:
            #         temp[y, x] = clustered_image[y, source_x]

            # Update shifted_image only where mask is true
            shifted_image[temp > 0] = temp[temp > 0]

        return shifted_image
    
    def sync_callback(self, webcam_msg, tof_msg):
        """
        Callback function that receives synchronized image pairs
        """
        try:
            # Decode compressed images
            webcam_np = np.frombuffer(webcam_msg.data, np.uint8)
            webcam_img = cv2.imdecode(webcam_np, cv2.IMREAD_COLOR)

            tof_np = np.frombuffer(tof_msg.data, np.uint8)
            # tof_img = cv2.imdecode(tof_np, cv2.IMREAD_GRAYSCALE)
            tof_img = cv2.imdecode(tof_np, cv2.IMREAD_COLOR)

            if webcam_img is None or tof_img is None:
                self.get_logger().warn('Failed to decode one or both images')
                return

            # Log timestamp difference for monitoring
            webcam_time = webcam_msg.header.stamp.sec + webcam_msg.header.stamp.nanosec * 1e-9
            tof_time = tof_msg.header.stamp.sec + tof_msg.header.stamp.nanosec * 1e-9
            time_diff = abs(webcam_time - tof_time)

            # self.get_logger().info(
            #     f'Synchronized images - Time diff: {time_diff*1000:.2f}ms, '
            #     f'Webcam: {webcam_img.shape}, ToF: {tof_img.shape}'
            # )

            # Crop top 15% of webcam image
            webcam_height = webcam_img.shape[0]
            crop_pixels = int(webcam_height * 0.15)
            webcam_cropped = webcam_img[crop_pixels:, :, :]

            # Scale cropped webcam image to match ToF image size
            tof_height, tof_width = tof_img.shape[:2]
            webcam_scaled = cv2.resize(
                webcam_cropped,
                (tof_width, tof_height),
                interpolation=cv2.INTER_LINEAR
            )

            # Apply median filter to ToF image (kernel size 5)
            

            # Convert ToF image to grayscale to compute distance
            tof_gray = cv2.cvtColor(tof_img, cv2.COLOR_BGR2GRAY)

            tof_filtered = cv2.medianBlur(tof_gray, 5)
            # tof_gray = tof_filtered

            # Compute distance image: distance = (255.001 - gray_level) * 400 / 255
            # distance_img = (255.001 - tof_gray.astype(np.float32)) * 200.0 / 255.0

            # Perform k-means clustering on distance image
            n_clusters = 8
            # distance_flat = distance_img.reshape(-1, 1)
# 
            # Use k-means clustering
            clustered_image, cluster_centers, dists = self.do_kmeans(tof_filtered,n_samples=n_clusters)

            #sort cluster centers
            cluster_centers = np.sort(cluster_centers.flatten())

            shifted_image = self.get_shifted_image(clustered_image, cluster_centers)

            # self.get_logger().info(f'Cluster centers (sorted): {np.sort(cluster_centers.flatten())}')
            lbls,stats,segmentation_masks = self.get_segmentation(clustered_image, cluster_centers)
            dists = ((255.0-np.array(lbls))*200.0/255.001)

            # self.get_logger().info(f'n_clusters={n_clusters}, len(lbls)={len(lbls)}, len(stats)={len(stats)}, len(segmentation_masks)={len(segmentation_masks)}')

            # shifted_image = np.zeros_like(clustered_image, dtype=np.float32)

            shifts = np.zeros_like(dists, dtype=np.float32)

            slope_near, intercept_near = self.interp_params[0]
            shifts[dists<40.0] = slope_near * (1.0 / dists[dists<40.0]) + intercept_near
            slope_far, intercept_far = self.interp_params[1]
            shifts[dists>=40.0] = slope_far * (1.0 / dists[dists>=40.0]) + intercept_far

            # Check if we have segmentation masks (some clusters may be filtered out)
            if len(segmentation_masks) == 0:
                self.get_logger().warn('No segmentation masks found (all regions too small)')
                overlay = webcam_scaled
            else:
                # Only use shifts for the actual number of segments we have
                num_segments = len(segmentation_masks)
                shifts_to_use = shifts[:num_segments]

                shifted_masks = []
                shifted_stats = []
                yn = webcam_scaled.shape[0]
                xn = webcam_scaled.shape[1]
                for i, p_val in enumerate(shifts_to_use):
                    try:
                        # self.get_logger().info(f'Segment {i} shift: {p_val:.2f} pixels')
                        st = np.copy(stats[i])
                        st[0] = st[0]+int(p_val)
                        st[2] = min(xn, st[2])
                        #tack this onto the left side of the mask
                        left_side = np.zeros((yn,int(p_val)), dtype=bool)
                        s_mask = np.concatenate((left_side,segmentation_masks[i]),axis=1)
                        s_mask = s_mask[:yn,:xn]
                        shifted_masks.append(s_mask)
                        shifted_stats.append(st)
                        # self.get_logger().info(f'two {i} shift: {p_val:.2f} pixels')
                    except IndexError as e:
                        self.get_logger().error(f'IndexError at i={i}, len(stats)={len(stats)}, len(segmentation_masks)={len(segmentation_masks)}, len(shifts_to_use)={len(shifts_to_use)}')

                img_out = webcam_scaled.copy()
                # self.draw_instance_segmentation_masks(img_out, shifted_masks)

                # Run YOLOv11 detection on webcam_scaled
                results = self.yolo_model(webcam_scaled, verbose=False)

                # List to collect all detections for publishing
                detections_list = []

                # Process each detection
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

                        #take the middle of the box as the detection point
                        x_center = int((x1 + x2) / 2)
                        y_center = int((y1 + y2) / 2)

                        if x2 - x1 > 111:
                            x1 = x_center - 55
                            x2 = x_center + 55
                        if y2 - y1 > 111:
                            y1 = y_center - 55
                            y2 = y_center + 55

                        conf = float(box.conf[0])
                        #round conf to 2 decimal places
                        conf = round(conf, 2)
                        cls = int(box.cls[0])

                        # Create detection mask for overlap calculation
                        det_mask = np.zeros((webcam_scaled.shape[0], webcam_scaled.shape[1]), dtype=bool)
                        det_mask[y1:y2, x1:x2] = True

                        # Find shifted_mask with biggest overlap
                        max_overlap = 0
                        best_idx = -1
                        for idx, shifted_mask in enumerate(shifted_masks):
                            overlap = np.sum(det_mask & shifted_mask)
                            if overlap > max_overlap:
                                max_overlap = overlap
                                best_idx = idx

                        # Get distance from dists array
                        if best_idx >= 0:
                            distance = dists[best_idx]

                            # Store detection data for publishing
                            object_name = self.yolo_model.names[cls]
                            detections_list.append({
                                'class': object_name,
                                'distance_cm': float(distance),
                                'confidence': conf,
                                'x_center': x_center,
                                'y_center': y_center,
                                'xn': xn,
                                'yn': yn
                            })

                            # Draw bounding box on img_out
                            cv2.rectangle(img_out, (x1, y1), (x2, y2), (0, 255, 0), 2)

                            # Draw distance label
                            label = f"{object_name} {distance:.1f}cm"
                            label = f"{conf} {distance:.1f}cm"
                            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                            cv2.rectangle(img_out, (x1, y1 - label_size[1] - 10), (x1 + label_size[0], y1), (0, 255, 0), -1)
                            cv2.putText(img_out, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                # Publish detections as JSON string with no truncation
                if detections_list:
                    detections_msg = String()
                    # Use separators to minimize whitespace for shorter message
                    detections_msg.data = json.dumps(detections_list, separators=(',', ':'))
                    self.detections_pub.publish(detections_msg)

                overlay = img_out


            # kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init=10)
            # labels = kmeans.fit_predict(distance_flat)
            # cluster_centers = kmeans.cluster_centers_.flatten()

            # # Create clustered distance image
            # distance_img_clustered = labels.reshape(distance_img.shape)

            # # Sort cluster centers from largest (furthest) to smallest (nearest)
            # sorted_indices = np.argsort(cluster_centers)[::-1]  # descending order
            # sorted_centers = cluster_centers[sorted_indices]

            # # Compute pixel shift for each cluster center
            # cluster_shifts = np.zeros(n_clusters)
            # for i, center in enumerate(sorted_centers):
            #     if center < 38.0:  # near
            #         slope_near, intercept_near = self.interp_params[0]
            #         cluster_shifts[i] = slope_near * (1.0 / center) + intercept_near
            #     else:  # far
            #         slope_far, intercept_far = self.interp_params[1]
            #         cluster_shifts[i] = slope_far * (1.0 / center) + intercept_far

            # # Create shifted ToF image by applying shifts per cluster
            # tof_shifted = tof_filtered.copy()

            # for i, center_idx in enumerate(sorted_indices):
            #     # Create mask for this cluster
            #     mask = (distance_img_clustered == center_idx)
            #     shift = int(round(cluster_shifts[i]))

            #     # Apply negative shift to the tof image for this cluster
            #     if shift != 0:
            #         # Create shifted version for this cluster
            #         temp = np.zeros_like(tof_filtered)
            #         y_coords, x_coords = np.where(mask)

            #         for y, x in zip(y_coords, x_coords):
            #             source_x = x - shift  # negative shift
            #             if 0 <= source_x < tof_filtered.shape[1]:
            #                 temp[y, x] = tof_filtered[y, source_x]

            #         # Update tof_shifted only where mask is true
            #         tof_shifted[mask] = temp[mask]

            # # Create shifted webcam overlay
            # # For each pixel in tof image, sample from webcam at position shifted to the right
            # height, width = tof_filtered.shape[:2]
            # webcam_shifted = np.zeros_like(webcam_scaled)



            # Create overlay with 0.3/0.7 transparency using shifted tof image
            # log shifted image shape and webcam_scaled shape
            # self.get_logger().info(f'shifted_image shape: {shifted_image.shape}, webcam_scaled shape: {webcam_scaled.shape}')
            #
            overlay_combined = self.draw_instance_segmentation_masks_gl(img_out, shifted_masks, dists)
            # shifted_image_color = cv2.cvtColor(np.uint8(shifted_image), cv2.COLOR_GRAY2BGR)
            # overlay_combined = cv2.addWeighted(
            #     webcam_scaled, 0.3,
            #     shifted_image_color, 0.7,
            #     0
            # )

            # overlay = cv2.addWeighted(
            #     webcam_scaled, 0.3,
            #     shifted_image, 0.7,
            #     0
            # )

            overlay_combined_msg = self.bridge.cv2_to_imgmsg(overlay_combined, encoding='bgr8')
            overlay_combined_msg.header = tof_msg.header  # Use ToF timestamp for output
            self.overlay_combined_pub.publish(overlay_combined_msg)

            # Publish the overlay image (uncompressed)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header = tof_msg.header  # Use ToF timestamp for output
            # self.overlay_pub.publish(overlay_msg)

            # Publish the overlay image (compressed)
            _, compressed_data = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, 90])
            overlay_compressed_msg = CompressedImage()
            overlay_compressed_msg.header = tof_msg.header
            overlay_compressed_msg.format = "jpeg"
            overlay_compressed_msg.data = compressed_data.tobytes()
            self.overlay_compressed_pub.publish(overlay_compressed_msg)

            _, compressed_combined_data = cv2.imencode('.jpg', overlay_combined, [cv2.IMWRITE_JPEG_QUALITY, 90])
            overlay_combined_compressed_msg = CompressedImage()
            overlay_combined_compressed_msg.header = tof_msg.header
            overlay_combined_compressed_msg.format = "jpeg"
            overlay_combined_compressed_msg.data = compressed_combined_data.tobytes()
            self.overlay_combined_compressed_pub.publish(overlay_combined_compressed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSyncProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
