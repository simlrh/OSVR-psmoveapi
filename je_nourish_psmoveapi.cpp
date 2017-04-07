/** @file
	@brief Comprehensive example: Implementation of a dummy Hardware Detect
	Callback that creates a dummy device when it is "detected"

	@date 2014

	@author
	Sensics, Inc.
	<http://sensics.com/osvr>
	*/

// Copyright 2014 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/Util/EigenInterop.h>
#include <osvr/Util/Pose3C.h>

// Generated JSON header file
#include "je_nourish_psmovecamera_json.h"
#include "je_nourish_psmovecontroller_json.h"

// Library/third-party includes
//
#include "psmove.h"
#include "psmove_tracker.h"
#include "psmove_fusion.h"
#include "psmove_vector.h"

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"

// Standard includes
#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <vector>

#define STABILIZE_WAIT_TIME_MS 1000

// Anonymous namespace to avoid symbol collision
namespace {

	typedef Eigen::Transform<float, 3, Eigen::Affine> Transform;

	Eigen::Quaternionf OpenGL_to_Eigen_Quaternion(const float* OpenGL_in) {
		float m[4][4];
		std::memcpy(m, OpenGL_in, 16 * sizeof(float));

		float w = sqrt(1 + m[0][0] + m[1][1] + m[2][2]) / 2;
		float x = (m[2][1] - m[1][2]) / (4 * w);
		float y = (m[0][2] - m[2][0]) / (4 * w);
		float z = (m[1][0] - m[0][1]) / (4 * w);

		Eigen::Quaternionf eigenQuat(w, x, y, z);
		return eigenQuat;
	}

	template<class Vector3>
		std::pair<Vector3, Vector3> best_plane_from_points(const std::vector<Vector3> & c)
		{
			size_t num_atoms = c.size();
			Eigen::Matrix< typename Vector3:: Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
			for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

			Vector3 centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

			coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

			auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
			Vector3 plane_normal = svd.matrixU().rightCols(1);
			return std::make_pair(centroid, plane_normal);
		}

	bool is_move_stable_and_aligned_with_gravity(PSMove *move) {
		const float k_cosine_10_degrees = 0.984808f;

		PSMove_3AxisVector k_identity_gravity_vector;
		PSMove_3AxisVector acceleration_direction;
		float acceleration_magnitude;
		bool isOk;

		// Get the direction the gravity vector should be pointing 
		// while the controller is in cradle pose.
		psmove_get_identity_gravity_calibration_direction(move, &k_identity_gravity_vector);
		psmove_get_accelerometer_frame(move, Frame_SecondHalf, 
				&acceleration_direction.x, &acceleration_direction.y, &acceleration_direction.z);
		acceleration_magnitude = psmove_3axisvector_normalize_with_default(&acceleration_direction, k_psmove_vector_zero);

		isOk =
			is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
			psmove_3axisvector_dot(&k_identity_gravity_vector, &acceleration_direction) >= k_cosine_10_degrees;

		return isOk;
	}

	class PSMoveController {
		public:
			PSMoveController(
					OSVR_PluginRegContext ctx, 
					PSMoveFusion *fusion, 
					PSMoveTracker *tracker,
					int i,
					Transform calibration
				) 
				: m_fusion(fusion)
				, m_calibration(calibration)
			{
				m_move = psmove_connect_by_id(i);

				psmove_enable_orientation(m_move, PSMove_True);
				assert(psmove_has_orientation(m_move));

				while (psmove_tracker_enable(tracker, m_move) != Tracker_CALIBRATED) {};

				std::string controllerName = "PSMoveController";
				std::string controllerID = std::to_string(i);
				std::string deviceName = controllerName + controllerID;

				OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);
				osvrDeviceTrackerConfigure(opts, &m_tracker);

				m_device.initSync(ctx, deviceName, opts);
				m_device.sendJsonDescriptor(je_nourish_psmovecontroller_json);
				m_device.registerUpdateCallback(this);
			}

			~PSMoveController() {
				psmove_disconnect(m_move);
			}

			OSVR_ReturnCode update() {
				while (psmove_poll(m_move));

				OSVR_PoseState pose;

				float x, y, z;
				psmove_fusion_get_position(m_fusion, m_move, &x, &y, &z);

				float* modelView = psmove_fusion_get_modelview_matrix(m_fusion, m_move);

				Eigen::Matrix4f modelViewMatrix;
				modelViewMatrix << modelView[0], modelView[1], modelView[2], x,
					modelView[4], modelView[5], modelView[6], -y,
					modelView[8], modelView[9], modelView[10], z,
					modelView[12], modelView[13], modelView[14], modelView[15];

				Eigen::Matrix4f calibratedMatrix = m_calibration * modelViewMatrix;

				Eigen::Matrix4f scaledMatrix = Eigen::Scaling(Eigen::Vector4f(0.1, 0.1, 0.1, 0.1)) * calibratedMatrix;

				osvr::util::toPose(scaledMatrix.cast<double>(), pose);

				osvrDeviceTrackerSendPose(m_device, m_tracker, &pose, 0);

				int buttons = psmove_get_buttons(m_move);

				unsigned int pressed, released;
				psmove_get_button_events(m_move, &pressed, &released);

				return OSVR_RETURN_SUCCESS;
			}


		private:
			Transform m_calibration;

			PSMove *m_move;
			PSMoveFusion *m_fusion;

			OSVR_Vec3 m_offset;

			osvr::pluginkit::DeviceToken m_device;
			OSVR_TrackerDeviceInterface m_tracker;
	};

	class PSMoveHandler {
		public:
			PSMoveHandler() 
				: m_initialised(false) 
				, m_sent_pose(false)
				, m_connected(psmove_count_connected())
				, m_tracker(psmove_tracker_new())
				, m_fusion(psmove_fusion_new(m_tracker, 1., 1000.))
			{ 
				psmove_tracker_set_exposure(m_tracker, Exposure_LOW);

				PSMove *move = psmove_connect_by_id(0);
				while (psmove_tracker_enable(m_tracker, move) != Tracker_CALIBRATED);


				int step = 5;
				std::vector<Eigen::Vector3f> points;
				Eigen::Vector3f origin;

				while (step >= 0) {
					if (step > 0) {
						std::cout << "Place controller at position " << (6 - step) << " and press the Move button" << std::endl;
					} else {
						std::cout << "Place controller at the origin and press the Move button" << std::endl;
					}
					bool step_complete = false;
					
					while (!step_complete) {
						psmove_tracker_update_image(m_tracker);
						psmove_tracker_update(m_tracker, NULL);
						psmove_tracker_annotate(m_tracker);

						frame = psmove_tracker_get_frame(m_tracker);
						if (frame) {
							cvShowImage("Calibration", frame);
							cvWaitKey(1);
						}

						while (psmove_poll(move));

						unsigned int pressed, released;
						psmove_get_button_events(move, &pressed, &released);

						if (pressed & Btn_MOVE) {
							/*
							bool stable = false;
							int stable_start_time = psmove_util_get_ticks();
							while (!stable) {
								if (psmove_poll(move)) {
									if (is_move_stable_and_aligned_with_gravity(move)) {
										int current_time = psmove_util_get_ticks();
										int stable_duration = (current_time - stable_start_time);

										if ((current_time - stable_start_time) >= STABILIZE_WAIT_TIME_MS) {
											stable = true;
										} else {
											printf("\rStable for: %dms/%dms                       ", stable_duration, STABILIZE_WAIT_TIME_MS);
										}
									} else {
										stable_start_time = psmove_util_get_ticks();
										printf("\rMove Destabilized! Waiting for stabilization.");
									}
								}
							}

							printf("\n");
							*/

							psmove_tracker_update_image(m_tracker);
							psmove_tracker_update(m_tracker, NULL);
							psmove_tracker_annotate(m_tracker);

							frame = psmove_tracker_get_frame(m_tracker);
							if (frame) {
								cvShowImage("Calibration", frame);
								cvWaitKey(1);
							}

							while (psmove_poll(move));

							float x, y, z;
							psmove_fusion_get_position(m_fusion, move, &x, &y, &z);

							if (step > 0) {
								points.emplace_back(x, y, z);
							} else {
								origin(0) = x;
								origin(1) = y;
								origin(2) = z;
							}

							step_complete = true;
						}
					}

					step -= 1;
				}

				std::pair<Eigen::Vector3f, Eigen::Vector3f> plane = best_plane_from_points(points);
				Eigen::Vector3f normal = plane.second;
				normal(1) = fabs(normal(1));
				Eigen::Vector3f trueNormal(0,1,0);

				Eigen::Quaternionf rotation = Eigen::Quaternionf().setFromTwoVectors(normal, trueNormal);
				Eigen::Translation3f translation(-origin(0), origin(1), -origin(2));

				std::cout << "Calibration:" << std::endl;
				std::cout << origin(0) << ", " << origin(1) << ", " << origin(2) << std::endl;
				std::cout << normal(0) << ", " << normal(1) << ", " << normal(2) << std::endl;

				m_calibration = rotation * translation;

				psmove_tracker_disable(m_tracker, move);
				psmove_disconnect(move);
			}

			~PSMoveHandler() {
				psmove_fusion_free(m_fusion);
				psmove_tracker_free(m_tracker);
			}

			// Hardware detection callback
			OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

				if (!m_initialised) {
					// Detect self
					OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

					osvrDeviceTrackerConfigure(opts, &m_camera_tracker);

					m_camera_device.initSync(ctx, "PSMoveCamera", opts);
					m_camera_device.sendJsonDescriptor(je_nourish_psmovecamera_json);
					m_camera_device.registerUpdateCallback(this);

					// Register controllers
					for (int i=0; i<m_connected; i++) {
						m_controllers[i] = new PSMoveController(ctx, m_fusion, m_tracker, i, m_calibration);
						osvr::pluginkit::registerObjectForDeletion(ctx, m_controllers[i]); 
					}

					m_initialised = true;
				}

				return OSVR_RETURN_SUCCESS;
			}

			// Update OSVR
			OSVR_ReturnCode update() {
				if (!m_sent_pose) {
					OSVR_PoseState pose;
					osvrPose3SetIdentity(&pose);
					osvrDeviceTrackerSendPose(m_camera_device, m_camera_tracker, &pose, 0);

					m_sent_pose = true;
				}

				psmove_tracker_update_image(m_tracker);
				psmove_tracker_update(m_tracker, NULL);
				psmove_tracker_annotate(m_tracker);

				frame = psmove_tracker_get_frame(m_tracker);
				if (frame) {
					cvShowImage("Calibration", frame);
					cvWaitKey(1);
				}

				return OSVR_RETURN_SUCCESS;
			}

		private:
			void *frame;

			int m_connected;
			PSMoveTracker *m_tracker;
			PSMoveFusion *m_fusion;

			Transform m_calibration;

			PSMoveController *m_controllers[PSMOVE_TRACKER_MAX_CONTROLLERS];

			bool m_initialised;
			bool m_sent_pose;
			osvr::pluginkit::DeviceToken m_camera_device;
			OSVR_TrackerDeviceInterface m_camera_tracker;
	};
} // namespace

OSVR_PLUGIN(com_osvr_example_selfcontained) {
	osvr::pluginkit::PluginContext context(ctx);

	PSMoveHandler *handler = new PSMoveHandler();

	context.registerHardwareDetectCallback(handler);

	return OSVR_RETURN_SUCCESS;
}
