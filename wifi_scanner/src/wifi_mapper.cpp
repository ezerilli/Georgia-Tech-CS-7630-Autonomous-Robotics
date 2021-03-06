
#include <vector>
#include <string>
#include <map>
#include <list>


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <wpa_cli/Scan.h>
#include <wpa_cli/Network.h>
#include <smart_battery_msgs/SmartBatteryStatus.h>




#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_WSIZE 800
#define WIN_HSIZE 700


class OccupancyGridPlanner {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber og_sub_;
        ros::Subscriber target_sub_;
        ros::Subscriber signal_sub_;
        ros::Subscriber reached_sub_;
        ros::Subscriber battery_sub_;
        ros::Subscriber mux_sel_sub_;
        ros::Publisher path_pub_;
        ros::Publisher target_pub_;
        ros::Publisher reached_pub_;
        //ros::Publisher mux_pub_;
        ros::Publisher undocker_pub_;
        tf::TransformListener listener_;

        cv::Rect roi_;
        cv::Mat_<uint8_t> og_, tg_, fg_;
        cv::Mat_<cv::Vec3b> og_rgb_, tg_rgb_, fg_rgb_, cropped_og_, cropped_tg_, cropped_fg_;
        cv::Point3i og_center_, target_ext, start_ext, current_point, origin;

        nav_msgs::MapMetaData info_;
        std::string frame_id_;
        std::string base_link_;
        std::vector<cv::Point3i> frontier;  
        bool ready, debug, first_run, first_target, docking;
        double radius, current_yaw;  
        int min_signal, max_signal, battery_charge, battery_threshold_high, battery_threshold_low; 
        std_msgs::Header reached;
        std_msgs::String mux_sel;
        std::string mux_selected;

        typedef std::multimap<float, cv::Point3i> Heap;
		
		// Callback for Mux Selected
        void mux_selected_callback(const std_msgs::StringConstPtr & msg) {
			mux_selected= msg->data;
			}
		
        // Callback for Occupancy Grids
        void og_callback(const nav_msgs::OccupancyGridConstPtr & msg) {
            info_ = msg->info;
            frame_id_ = msg->header.frame_id;
            // Create an image to store the value of the grid.
            og_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);
            og_center_ = cv::Point3i(-info_.origin.position.x/info_.resolution,
                    -info_.origin.position.y/info_.resolution,0);
            ROS_INFO("Og_size (%i, %i)", msg->info.height, msg->info.width);

			if (first_run){
				tg_ = cv::Mat_<uint8_t>(og_.size(), 0x40);
				fg_ = cv::Mat_<uint8_t>(og_.size(), 0xFF);
				cv::cvtColor(fg_, fg_rgb_, CV_GRAY2RGB);
				first_run=false;
			}
            // Some variables to select the useful bounding box 
            unsigned int maxx=0, minx=msg->info.width, 
                         maxy=0, miny=msg->info.height;
            // Convert the representation into something easy to display.
            for (unsigned int j=0;j<msg->info.height;j++) {
                for (unsigned int i=0;i<msg->info.width;i++) {
                    int8_t v = msg->data[j*msg->info.width + i];
                    if(v>90 && v<=100){
						og_(j,i) = OCCUPIED;
					} else {
						og_(j,i) = FREE; 
					}
                    
                    // Update the bounding box of free or occupied cells.
                    if (og_(j,i) != UNKNOWN) {
                        minx = std::min(minx,i);
                        miny = std::min(miny,j);
                        maxx = std::max(maxx,i);
                        maxy = std::max(maxy,j);
                    }
                }
            }
            // Apply the dilation on obstacles (= erosion of black cells)
			double dilation_size =1.1*radius/info_.resolution;
			cv::Mat element = getStructuringElement( cv::MORPH_RECT,
								   cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
								   cv::Point( dilation_size, dilation_size ) );
			cv::erode( og_, og_, element );
			
            if (!ready) {
                ready = true;
                origin = current_position;
                ROS_INFO("Received occupancy grid, ready to plan");
            }
			
			// Make og_rgb_ plotting obstacles in red
            //cv::cvtColor(og_, og_rgb_, CV_GRAY2RGB);
            std::vector<cv::Mat> images(3);
			cv::Mat_<uint8_t> white = cv::Mat_<uint8_t>(og_.size(),0xFF);
			images.at(0) = og_; //for blue channel
			images.at(1) = og_; //for green channel
			images.at(2) = white;  //for red channel
			cv::merge(images, og_rgb_);    
        }
		
		
        // Generic test if a point is within the occupancy grid
        bool isInGrid(const cv::Point3i & P) {
            if ((P.x < 0) || (P.x >= (signed)info_.width) 
                    || (P.y < 0) || (P.y >= (signed)info_.height)) {
                return false;
            }
            return true;
        }
        
        double heuristic(const cv::Point3i & currP, const cv::Point3i & goalP) {
            return hypot(goalP.x - currP.x, goalP.y - currP.y);
		}
		
		cv::Point point3iToPoint(const cv::Point3i & currPoint) {
            return cv::Point(currPoint.x, currPoint.y);
		}
        
        // This is called when a new goal is posted by RViz. We don't use a
        // mutex here, because it can only be called in spinOnce.
        void target_callback(const geometry_msgs::PoseStampedConstPtr & msg) {
            tf::StampedTransform transform;
            geometry_msgs::PoseStamped pose;
            if (!ready) {
                ROS_WARN("Ignoring target while the occupancy grid has not been received");
                return;
            }
            
            if (!docking) {
                ROS_INFO("Auto Docking is running");
                return;
            }
            ROS_INFO("Received planning request");
            // Convert the destination point in the occupancy grid frame. 
            // The debug case is useful if the map is published without
            // gmapping running (for instance with map_server).
            if (debug) {
                pose = *msg;
            } else {
                // This converts target in the grid frame.
                listener_.waitForTransform(frame_id_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
                listener_.transformPose(frame_id_,*msg, pose);
                // this gets the current pose in transform
                listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
            }
            // Now scale the target to the grid resolution and shift it to the
            // grid center.
            double t_yaw = tf::getYaw(pose.pose.orientation);
            cv::Point3i target = cv::Point3i(pose.pose.position.x / info_.resolution, pose.pose.position.y / info_.resolution, (unsigned int)(round(t_yaw / (M_PI/4))) % 8)
					+ og_center_;
            target_ext=target;  
            
			//ROS_INFO("Planning target: %.2f %.2f %.2f -> %d %d %d",
			//		pose.pose.position.x, pose.pose.position.y, t_yaw, target.x, target.y, target.z);
			
            if (!isInGrid(target)) {
                ROS_INFO("Planning target outside the grid: %.2f %.2f %.2f -> %d %d %d",
                        pose.pose.position.x, pose.pose.position.y, t_yaw, target.x, target.y, target.z);
                return;
            }
            // Only accept target which are FREE in the grid 
            if (og_(point3iToPoint(target)) != FREE) {
                ROS_ERROR("Invalid target point: occupancy = %d",og_(point3iToPoint(target)));
                return;
            }

            // Now get the current point in grid coordinates.
            cv::Point3i start;
            double s_yaw = 0;
            if (debug) {
                start = og_center_;
            } else {
				s_yaw = tf::getYaw(transform.getRotation());
                start = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution, (unsigned int)(round(s_yaw / (M_PI/4))) % 8)
                    + og_center_;
            }
            start_ext=start;
            
            //ROS_INFO("Planning origin %.2f %.2f %.2f -> %d %d %d",
            //        transform.getOrigin().x(), transform.getOrigin().y(), s_yaw, start.x, start.y, start.z);
            if (!isInGrid(start)) {
               ROS_INFO("Planning origin outside the grid %.2f %.2f %.2f -> %d %d %d",
                    transform.getOrigin().x(), transform.getOrigin().y(), s_yaw, start.x, start.y, start.z);
                return;
            }
            // If the starting point is not FREE there is a bug somewhere, but
            // better to check
            if (og_(point3iToPoint(start)) != FREE) {
                ROS_ERROR("Invalid start point: occupancy = %d",og_(point3iToPoint(start)));
				for (int m=-2; m<3; m++){
					for (int n=-2; n<3; n++){
						og_(start.y+m,start.x+n)=FREE;
					}	
				}
                return;
            }
            
            if(battery_charge<=battery_threshold_low){
				    target=starting_point;
				    target_ext=target;
				if(hypot(start.x-target.x, start.y-target.y)<=10){
					ROS_ERROR("Auto docking");
					docking=false;
					first_target=true;
					
					nav_msgs::Path void_path;
					path_pub_.publish(void_path);
					
					std::system("rosnode kill /obstacle_avoidance&");
					ros::Duration(1.0).sleep();
					std:system("roslaunch kobuki_auto_docking activate.launch");
					ros::Duration(1.0).sleep();
					return;
				} else {
					ROS_ERROR("Coming home : from (%d, %d, %d) to (%d, %d, %d)",start.x,start.y,start.z, target.x, target.y, target.z);
				}
			} else {
				ROS_INFO("Starting planning from (%d, %d, %d) to (%d, %d, %d)",start.x,start.y,start.z, target.x, target.y, target.z);
			}
			
            // Here the Dijskstra algorithm starts 
            // The best distance to the goal computed so far. This is
            // initialised with Not-A-Number. 
            int dim[3]={og_.size().width, og_.size().height, 8};
            cv::Mat_<float> cell_value(3, dim, NAN);
            // For each cell we need to store a pointer to the coordinates of
            // its best predecessor. 
            cv::Mat_<cv::Vec3s> predecessor(3, dim);

            // The neighbour of a given cell in relative coordinates. The order
            // is important. If we use 4-connexity, then we can use only the
            // first 4 values of the array. If we use 8-connexity we use the
            // full array.
			cv::Point3i neighbours[8][5]={
				{cv::Point3i( 1, 0,0), cv::Point3i( 1, 1,1), cv::Point3i( 1,-1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
				{cv::Point3i( 1, 1,0), cv::Point3i( 0, 1,1), cv::Point3i( 1, 0,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
				{cv::Point3i( 0, 1,0), cv::Point3i(-1, 1,1), cv::Point3i( 1, 1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
				{cv::Point3i(-1, 1,0), cv::Point3i(-1, 0,1), cv::Point3i( 0, 1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
				{cv::Point3i(-1, 0,0), cv::Point3i(-1,-1,1), cv::Point3i(-1, 1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
				{cv::Point3i(-1,-1,0), cv::Point3i( 0,-1,1), cv::Point3i(-1, 0,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
				{cv::Point3i( 0,-1,0), cv::Point3i( 1,-1,1), cv::Point3i(-1,-1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
				{cv::Point3i( 1,-1,0), cv::Point3i( 1, 0,1), cv::Point3i( 0,-1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)}
				};
				
            // Cost of displacement corresponding the neighbours. Diagonal
            // moves are 44% longer.
			float cost[2][5] = {{      1, 2*sqrt(2), 2, 100, 100},
							    {sqrt(2), 		  2, 2, 100, 100}};
            
            
            // The core of Dijkstra's Algorithm, a sorted heap, where the first
            // element is always the closer to the start.
            Heap heap;
            heap.insert(Heap::value_type(heuristic(start,target), start));
            cell_value(start.x,start.y,start.z) = 0;
            while (!heap.empty() && !(heap.begin()->second==target)) {
                // Select the cell at the top of the heap
                Heap::iterator hit = heap.begin();
                // the cell hit contains is this_cell
                cv::Point3i this_cell = hit->second;
                // and its score is this_cost
                float this_cost = hit->first;
                
                // We can remove it from the heap now.
                heap.erase(hit);
                // Now see where we can go from this_cell
                for (unsigned int i=0;i<5;i++) {
					cv::Point3i dest = this_cell + neighbours[this_cell.z][i];
					dest.z = (dest.z+8)%8;
					if (!isInGrid(dest)) {
						// outside the grid
						continue;
					}
					uint8_t og = og_(point3iToPoint(dest));
					if (og != FREE) {
						// occupied or unknown
						continue;
					}
					float cv = cell_value(dest.x,dest.y,dest.z);
					float new_cost = this_cost + ((dest.z&1)?(cost[0][i]):(cost[1][i]));

					if (isnan(cv) || (new_cost < cv)) {
						// found shortest path (or new path), updating the
						// predecessor and the value of the cell
						predecessor.at<cv::Vec3s>(dest.x,dest.y,dest.z) = cv::Vec3s(this_cell.x,this_cell.y,this_cell.z);
						cell_value(dest.x,dest.y,dest.z)= new_cost;
						// And insert the selected cells in the map.
						heap.insert(Heap::value_type(new_cost+heuristic(dest,target),dest));
						//ROS_INFO("dest (%d %d %d) to target (%d, %d, %d), cost %d",dest.x,dest.y,dest.z,target.x,target.y,target.z,new_cost+heuristic(dest,target));
					}
                }
            }
            if (isnan(cell_value(target.x, target.y, target.z))) {
                // No path found
                ROS_ERROR("No path found from (%d, %d, %d) to (%d, %d, %d) because ",start.x,start.y,start.z,target.x,target.y,target.z);
                return;
            }
            ROS_INFO("Planning completed");
            // Now extract the path by starting from goal and going through the
            // predecessors until the starting point
            std::list<cv::Point3i> lpath;
            int num_iter=0;
            while (target != start) {
                lpath.push_front(target);
                cv::Vec3s p = predecessor(target.x, target.y, target.z);
                target.x = p[0]; target.y = p[1], target.z=p[2];
                if (num_iter>10000000){
					ROS_ERROR("Ho buggato");
					return;
				}		
				num_iter++;
            }
            lpath.push_front(start);
            // Finally create a ROS path message
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = frame_id_;
            path.poses.resize(lpath.size());
            std::list<cv::Point3i>::const_iterator it = lpath.begin();
            unsigned int ipose = 0;
            while (it != lpath.end()) {
                // time stamp is not updated because we're not creating a
                // trajectory at this stage
                path.poses[ipose].header = path.header;
                cv::Point3i P = *it - og_center_;
                path.poses[ipose].pose.position.x = (P.x) * info_.resolution;
                path.poses[ipose].pose.position.y = (P.y) * info_.resolution;
                tf::Quaternion Q = tf::createQuaternionFromRPY(0,0,P.z*M_PI/4);
				tf::quaternionTFToMsg(Q,path.poses[ipose].pose.orientation);
                ipose++;
                it ++;
            }
            path_pub_.publish(path);
            ROS_INFO("Request completed");
        }
		
		// Callback for Treasure Grids
		void tg_callback(const wpa_cli::ScanConstPtr & msg) {
			if(!first_run){
				// this gets the current pose in transform
				tf::StampedTransform transform;
				listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
				// Computing robot's actual position
				if (debug) {
					current_point = og_center_;
				} else {
					current_yaw = tf::getYaw(transform.getRotation());
					current_point = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution, (unsigned int)(round(current_yaw / (M_PI/4))) % 8)
						+ og_center_;
				}
				// Plotting signal intensity on a circular area around the robot of radius r
				int signal=-100;
				for (int i=0; i<msg->networks.size();i++){
					if(msg->networks[i].ssid=="GTwifi" && msg->networks[i].level>signal){
						signal=msg->networks[i].level;
					}
				}
				if(max_signal<signal){
					max_signal=signal;
				}
				if(min_signal>signal){
					min_signal=signal;
				}
				
				// Scaling calculus
				// -40+x=y
				// -85+x=0.3*y
				//  45  =0.7*y => y=64, x=104
				
				float signalf=(signal+104.0)/64.0;								
				float max_signalf=(max_signal+104.0)/64.0;	
				float min_signalf=(min_signal+104.0)/64.0;	
				//ROS_INFO("Signal = %i (%.2f)",signal,signalf);
				//ROS_INFO("Max_Signal, Min_Signal = (%i = %.2f, %.i = %.2f)",max_signal, max_signalf, min_signal, min_signalf);

				double r;
				uint8_t intensity;
				for(int i=10;i>=-10;i--){
					for(int j=10;j>=-10;j--){
						intensity=(uint8_t)(signalf*FREE);
						cv::Point3i radius_point=cv::Point3i(i,j,0);
						 r = hypot(radius_point.x,radius_point.y);
						if(intensity > tg_(point3iToPoint(current_point+radius_point)) && r<=10) {
							tg_(point3iToPoint(current_point+radius_point))= intensity;
						}
					} 
				}
				
				// Computing the frontier known/unknown
				frontier.clear();
				fg_ = cv::Mat_<uint8_t>(og_.size(), 0xFF);
				cv::Size s = tg_rgb_.size();
				for (unsigned int j=0;j<s.height;j++) {
					for (unsigned int i=0;i<s.width;i++) {
						if (tg_rgb_(j,i).val[0]>0x40){
							for (int m=-1; m<2; m++){
								for (int n=-1; n<2; n++){
									if (tg_rgb_(j+n,i+m).val[0]==0x40){
										frontier.push_back(cv::Point3i(i,j,0));
										fg_(j,i)=OCCUPIED;
									}	
								}
							}			
						}	
					}
				}
				
				frontier.erase( unique( frontier.begin(), frontier.end() ), frontier.end() );
				
				// The lines below are only for display
				s = tg_rgb_.size();
				unsigned int w = s.width;
				unsigned int h = s.height;
				roi_ = cv::Rect(0,0,w,h);
				cv::cvtColor(tg_, tg_rgb_, CV_GRAY2RGB);
				tg_rgb_=tg_rgb_&og_rgb_;
				tg_rgb_(point3iToPoint(current_point)).val[1]=OCCUPIED;
				
				cv::cvtColor(fg_, fg_rgb_, CV_GRAY2RGB);
				fg_rgb_=fg_rgb_&tg_rgb_;
				
				for (int i=0; i<frontier.size(); i++){
					fg_rgb_(point3iToPoint(frontier[i])).val[1]=FREE;
				}
				//cv::circle(fg_rgb_,point3iToPoint(target_ext), 1, cv::Scalar(0,125,0));
				//cv::circle(fg_rgb_,point3iToPoint(start_ext), 1, cv::Scalar(255,255,0));
				fg_rgb_(point3iToPoint(target_ext)).val[1]=0xFF;
				fg_rgb_(point3iToPoint(target_ext)).val[2]=0xFF;

				fg_rgb_(point3iToPoint(start_ext)).val[0]=0xFF;
				fg_rgb_(point3iToPoint(start_ext)).val[1]=0xFF;

				// Compute a sub-image that covers only the useful part of the
				// grid.
				cropped_og_ = cv::Mat_<cv::Vec3b>(og_rgb_,roi_);
				cropped_tg_ = cv::Mat_<cv::Vec3b>(tg_rgb_,roi_);
				cropped_fg_ = cv::Mat_<cv::Vec3b>(fg_rgb_,roi_);
				if ((w > WIN_WSIZE) || (h > WIN_HSIZE)) {
					// The occupancy grid is too large to display. We need to scale
					// it first.
					double ratio = w / ((double)h);
					cv::Size new_size;
					if (ratio >= 1) {
						new_size = cv::Size(WIN_WSIZE,WIN_HSIZE/ratio);
					} else {
						new_size = cv::Size(WIN_WSIZE*ratio,WIN_HSIZE);
					}
					cv::Mat_<cv::Vec3b> resized_og, resized_tg, resized_fg;
					cv::resize(cropped_og_,resized_og,new_size);
					cv::resize(cropped_tg_,resized_tg,new_size);
					cv::resize(cropped_fg_,resized_fg,new_size);
					//cv::imshow( "OccGrid", resized_og );
					//cv::imshow( "TreasureGrid", resized_tg );
					cv::imshow( "FrontierGrid", resized_fg );
					
				} else {
					//cv::imshow( "OccGrid", og_rgb_ );
					//cv::imshow( "TreasureGrid", tg_rgb_ );
					cv::imshow( "FrontierGrid", fg_rgb_ );
				}
				if(first_target && battery_charge>=battery_threshold_high && mux_selected=="/mux/autoCommand") {
					
					ROS_ERROR("Undocking");
					geometry_msgs::Twist velocity;

					ros::Rate rate(10);
					ros::Time start(ros::Time::now());
					while (ros::ok()) {
						if ((ros::Time::now() - start).toSec() > 2.0) {
							break;
						}
						velocity.linear.x = -0.3;
						undocker_pub_.publish(velocity);
						rate.sleep();
					}
					
					velocity.linear.x = 0.0;
					undocker_pub_.publish(velocity);
					ros::Duration(1).sleep();
					
					start = ros::Time::now();
					while (ros::ok()) {
						if ((ros::Time::now() - start).toSec() > 5.5) {
							break;
						}
						
						velocity.angular.z = 1.0;
						undocker_pub_.publish(velocity);
						rate.sleep();
					}
					velocity.angular.z = 0.0;
					undocker_pub_.publish(velocity);
					
					ros::Duration(3).sleep();	
					
					ROS_INFO("First target given");
					first_target=false;
					docking=true;
					reached.stamp = ros::Time::now();
                    reached.frame_id = frame_id_;
					reached_pub_.publish(reached);
				}
			}
		}
		
		
		// Callback for Exploration
		void reached_callback(const std_msgs::Header & msg) {
			if(!first_run){	
				geometry_msgs::PoseStamped goal_pose;	
				goal_pose.header.stamp = ros::Time::now();
				goal_pose.header.frame_id = frame_id_;
				// this gets the current pose in transform
				tf::StampedTransform transform;
				listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
				
				if (debug) {
					current_point = og_center_;
				} else {
					current_yaw = tf::getYaw(transform.getRotation());
					current_point = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution, (unsigned int)(round(current_yaw / (M_PI/4))) % 8)
						+ og_center_;
				}
				
				int idx=0;
				float dpos, dtheta, curr_scr=0., best_scr= 1000000.;
				for (int i=0; i<frontier.size(); i++){
					dpos= hypot(frontier[i].x-current_point.x,frontier[i].y-current_point.y);
					dtheta=fabs(current_yaw-atan2(frontier[i].y-current_point.y,frontier[i].x-current_point.x));
					
					curr_scr=0.01*dpos*dpos+10000.0*dtheta;
					if(dpos>4*radius && curr_scr<best_scr && og_rgb_(frontier[i].y,frontier[i].x).val[0]!=0x00 && og_rgb_(frontier[i].y+2,frontier[i].x+2).val[0]!=0x00 && 
					   og_rgb_(frontier[i].y-2,frontier[i].x+2).val[0]!=0x00 && og_rgb_(frontier[i].y+2,frontier[i].x-2).val[0]!=0x00 && og_rgb_(frontier[i].y-2,frontier[i].x-2).val[0]!=0x00){
						
						best_scr=curr_scr;
						idx=i;
					}
				}
				
				if(best_scr!=1000000.){
					ROS_ERROR("BEST FOUND! Score = %.2f", best_scr);
					cv::Point3i new_goal =  frontier[idx] - og_center_;						
					goal_pose.pose.position.x = (new_goal.x) * info_.resolution;
					goal_pose.pose.position.y = (new_goal.y) * info_.resolution;
					tf::Quaternion Q = tf::createQuaternionFromRPY(0,0,atan2(frontier[idx].y-current_point.y,frontier[idx].x-current_point.x));
					tf::quaternionTFToMsg(Q,goal_pose.pose.orientation);
					target_pub_.publish(goal_pose);
				} else {
					ROS_ERROR("BEST NOT FOUND! %lu", frontier.size());
					reached.stamp = ros::Time::now();
                    reached.frame_id = frame_id_;
					reached_pub_.publish(reached);
					
				}
			}
		}
		
		// Callback for Battery Management
		void battery_callback(const smart_battery_msgs::SmartBatteryStatusConstPtr & msg) {
			battery_charge= msg->percentage;
			//ROS_INFO("Battery Level is %i percent", battery_charge);
		}


    public:
        OccupancyGridPlanner() : nh_("~"), ready(false), first_run(true), first_target(true), min_signal(0), max_signal(-100) , battery_charge(100), docking(true), mux_selected(""){
            nh_.param("base_frame",base_link_,std::string("/base_link"));
            nh_.param("debug",debug,false);
            nh_.param("radius",radius,0.25);
            nh_.param("battery_threshold_high",battery_threshold_high,90);
            nh_.param("battery_threshold_low",battery_threshold_low,20);
            og_sub_ = nh_.subscribe("occ_grid",1,&OccupancyGridPlanner::og_callback,this);
            target_sub_ = nh_.subscribe("goal",1,&OccupancyGridPlanner::target_callback,this);
            signal_sub_ = nh_.subscribe("signal",1,&OccupancyGridPlanner::tg_callback,this);
            reached_sub_= nh_.subscribe("goal_reached",1,&OccupancyGridPlanner::reached_callback,this);
            battery_sub_= nh_.subscribe("battery_charge",1,&OccupancyGridPlanner::battery_callback,this);
            mux_sel_sub_= nh_.subscribe("/mux/selected",1,&OccupancyGridPlanner::mux_selected_callback,this);
            path_pub_ = nh_.advertise<nav_msgs::Path>("path",1,true);
            target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal",1);
            reached_pub_ = nh_.advertise<std_msgs::Header>("goal_reached",1);
            //mux_pub_ = nh_.advertise<std_msgs::String>("/mux/selected",1);
            undocker_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1);
        }
};

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"occgrid_planner");
    OccupancyGridPlanner ogp;
    //cv::namedWindow( "OccGrid", CV_WINDOW_AUTOSIZE );
    while (ros::ok()) {
        ros::spinOnce();
        if (cv::waitKey( 50 )== 'q') {
            ros::shutdown();
        }
    }
}

