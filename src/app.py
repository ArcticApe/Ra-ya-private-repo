import argparse

from raya.application_base import RayaApplicationBase
from raya.controllers.cameras_controller import CamerasController
from raya.controllers.cv_controller import CVController, DetectionObjectsHandler
from raya.controllers.grasping_controller import GraspingController
from raya.controllers.arms_controller import ArmsController
from raya.controllers.motion_controller import MotionController
from raya.controllers.navigation_controller import NavigationController, POS_UNIT
from raya.tools.image import show_image
from raya.exceptions import RayaNavNotNavigating, RayaGraspingNotGrasping, RayaCVAlreadyEnabledType
from raya.enumerations import THEME_TYPE

MAP = 'unity_apartment'
CAMERA = 'head_front'
OBJECTS = 'cup'
MODEL = 'apartment_objects'
LOCATION1 = 'room01'
LOCATION2 = 'living_room'
ARM1 = 'right_arm'
ARM2 = 'left_arm'


class RayaApplication(RayaApplicationBase):

    ########################## START OF SETUP ##########################
    async def setup(self):
        self.arrived = False
        self.isGrasped = False
        self.arms_dict = {'right_arm' : [False, 0.0], 'left_arm' : [False, 0.0]}
        self.model_name = 'coral_efficientdet_lite0_320_coco'

        # Create local attributes and variables
        self.ui = await self.enable_controller('ui')
        self.camera: CamerasController = await self.enable_controller('cameras')
        await self.camera.enable_color_camera(CAMERA)
        self.cv: CVController = await self.enable_controller('cv')
        self.detector: DetectionObjectsHandler = await self.cv.enable_model(model='detectors', type='object',
                                                    name=self.model_name,
                                                    source=CAMERA,
                                                    model_params={})
        self.nav: NavigationController = await self.enable_controller('navigation')
        self.grasp: GraspingController = await self.enable_controller('grasping')
        self.arm: ArmsController = await self.enable_controller('arms')
        self.motion: MotionController = await self.enable_controller('motion')

        # Get arguments
        self.get_args()

         # Setting up the map
        self.log.info((f'Setting map: {self.map_name}. '
                       'Waiting for the robot to get localized'))
        if not await self.nav.set_map(self.map_name,
                                      wait_localization=True,
                                      timeout=3.0):
            self.log.info((f'Robot couldn\'t localize itself'))
            self.finish_app()

        self.status = await self.nav.get_status()
        self.log.info(f'status: {self.status}')

    def get_args(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('-m', '--map-name',
                            type=str,
                            default='unity_apartment', required=False,
                            help='Map name')
        parser.add_argument("-mm", "--model", 
                            help="object model name")
        args = parser.parse_args()

        self.map_name = args.map_name
        self.model = args.model

    def get_available_arms(self):
        arms = []
        for arm in self.arms_dict:
            if not self.arms_dict[arm][0]:
                arms.append(arm)
                
    def grasp_feedback_cb(self, state):
        print(f'Grasping feedback: {state}')
    
    def grasp_finished_cb(self, error, error_msg, height_object, arm_pick):
        if error:
            self.isGrasped = False
            self.log.info(f'Grasping finished: {error}, {error_msg}')
            self.log.info('Finish picking object')
            self.finish_app()
        else:
            self.isGrasped = True
            self.log.info(f'Grasping finished: height object{height_object} , Using {arm_pick} to grab the detedcted object.')
            self.log.info(f'Succesfully picked up the {OBJECTS} with {ARM1}')
            self.sleep(1.0)

    ########################## END OF SETUP ##########################

    ########################## START OF LOOP ##########################

    async def loop(self):
       await self.nav.navigate_to_location(
           zone_name=LOCATION1,
           callback_feedback=self.nav_callback_feedback,
           callback_finish=self.nav_callback_finish,
           wait=True
       )
       if self.arrived:
            self.log.info(f'Checking for objects')
            ##await self.motion.rotate(angle=360.0, angular_velocity=30.0, wait=True)
            resp = await self.detector.find_objects([OBJECTS], wait=True, timeout=40.0)
            if resp is not None:
                self.log.info(f'{OBJECTS} detected!')
                self.object_detected = True
                self.arrived = True

                object_x = resp[0]['center_point_map'][0]
                object_y = resp[0]['center_point_map'][1]

                await self.sleep(1.0)

                self.log.info(f'Navigating to {OBJECTS} position')
                await self.nav.navigate_close_to_position(x=object_x,
                                                          y=object_y,
                                                          pos_unit=POS_UNIT.METERS,
                                                          wait=True)

                self.log.info(f'Picking {OBJECTS} with {ARM1}')
                ## Obtain available arms to pick
                available_arms = self.get_available_arms()
                await self.grasp.pick_object(detector_model = self.model_name,
                                       source = CAMERA,
                                       object_name = OBJECTS,
                                       arms = available_arms,
                                       ##callback_feedback = self.grasp_feedback_cb,
                                      callback_finish = self.grasp_finished_cb,
                                       wait = True,
                                    )
            else:
                self.log.info(f'{OBJECTS} not detected!')
                self.object_detected = False
                self.arrived = True
                self.finish()
       else:
            self.log.info(f'Agent did not reach {self.location_name}')
            self.finish_app()
       if not self.object_detected:
         self.log.info(f'I reached {self.location_name} and I did not find any item')
         self.finish()
       else:
            self.log.info(f'Please tell me what to do next time.')
            self.finish_app()
                 
    ########################## END OF LOOP ##########################

    ########################## START OF FINISH ##########################
    async def finish(self):
        # Finishing instructions
        self.log.info(f'Turning off {CAMERA}')
        self.camera.disable_color_camera(CAMERA)
        
    ########################## END OF FINISH ##########################

    

    def nav_callback_feedback(self, state, distance_to_goal, speed):
        self.log.info(f'Feedback: {state}, {distance_to_goal}, {speed}')

    def nav_callback_finish(self, error, error_msg):
        if error:
            self.log.info(f'Finish: {error}, {error_msg}')
        else:
            self.log.info(f'I arrived at {LOCATION1}')
            self.arrived = True
    