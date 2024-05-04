from types import SimpleNamespace

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params

from openpilot.selfdrive.frogpilot.controls.lib.model_manager import RADARLESS_MODELS

class FrogPilotVariables:
  def __init__(self):
    self.frogpilot_toggles = SimpleNamespace()

    self.params = Params()
    self.params_memory = Params("/dev/shm/params")

    with car.CarParams.from_bytes(self.params.get("CarParams", block=True)) as msg:
      self.CP = msg

    self.frogpilot_toggles.radarless_model = self.params.get("Model", block=True, encoding='utf-8') in RADARLESS_MODELS

    self.update_frogpilot_params()

  def update_frogpilot_params(self):
    self.frogpilot_toggles.is_metric = self.params.get_bool("IsMetric")

    always_on_lateral = self.params.get_bool("AlwaysOnLateral")
    self.frogpilot_toggles.always_on_lateral_pause_speed = always_on_lateral and self.params.get_int("PauseAOLOnBrake")

    self.frogpilot_toggles.conditional_experimental_mode = self.CP.openpilotLongitudinalControl and self.params.get_bool("ConditionalExperimental")
    self.frogpilot_toggles.conditional_curves = self.frogpilot_toggles.conditional_experimental_mode and self.params.get_bool("CECurves")
    self.frogpilot_toggles.conditional_curves_lead = self.frogpilot_toggles.conditional_curves and self.params.get_bool("CECurvesLead")
    self.frogpilot_toggles.conditional_limit = self.params.get_int("CESpeed") * (CV.KPH_TO_MS if self.frogpilot_toggles.is_metric else CV.MPH_TO_MS) if self.frogpilot_toggles.conditional_experimental_mode else 0
    self.frogpilot_toggles.conditional_limit_lead = self.params.get_int("CESpeedLead") * (CV.KPH_TO_MS if self.frogpilot_toggles.is_metric else CV.MPH_TO_MS) if self.frogpilot_toggles.conditional_experimental_mode else 0
    self.frogpilot_toggles.conditional_navigation = self.frogpilot_toggles.conditional_experimental_mode and self.params.get_bool("CENavigation")
    self.frogpilot_toggles.conditional_navigation_intersections = self.frogpilot_toggles.conditional_navigation and self.params.get_bool("CENavigationIntersections")
    self.frogpilot_toggles.conditional_navigation_lead = self.frogpilot_toggles.conditional_navigation and self.params.get_bool("CENavigationLead")
    self.frogpilot_toggles.conditional_navigation_turns = self.frogpilot_toggles.conditional_navigation and self.params.get_bool("CENavigationTurns")
    self.frogpilot_toggles.conditional_signal = self.frogpilot_toggles.conditional_experimental_mode and self.params.get_bool("CESignal")
    self.frogpilot_toggles.conditional_slower_lead = self.frogpilot_toggles.conditional_experimental_mode and self.params.get_bool("CESlowerLead")
    self.frogpilot_toggles.conditional_stop_lights = self.frogpilot_toggles.conditional_experimental_mode and self.params.get_bool("CEStopLights")
    self.frogpilot_toggles.conditional_stop_lights_lead = self.frogpilot_toggles.conditional_stop_lights and self.params.get_bool("CEStopLightsLead")

    custom_alerts = self.params.get_bool("CustomAlerts")
    self.frogpilot_toggles.green_light_alert = custom_alerts and self.params.get_bool("GreenLightAlert")
    self.frogpilot_toggles.lead_departing_alert = not self.frogpilot_toggles.radarless_model and custom_alerts and self.params.get_bool("LeadDepartingAlert")
    self.frogpilot_toggles.loud_blindspot_alert = custom_alerts and self.params.get_bool("LoudBlindspotAlert")

    self.frogpilot_toggles.custom_personalities = self.params.get_bool("CustomPersonalities")
    self.frogpilot_toggles.aggressive_jerk = self.params.get_float("AggressiveJerk")
    self.frogpilot_toggles.aggressive_follow = self.params.get_float("AggressiveFollow")
    self.frogpilot_toggles.standard_jerk = self.params.get_float("StandardJerk")
    self.frogpilot_toggles.standard_follow = self.params.get_float("StandardFollow")
    self.frogpilot_toggles.relaxed_jerk = self.params.get_float("RelaxedJerk")
    self.frogpilot_toggles.relaxed_follow = self.params.get_float("RelaxedFollow")
    self.frogpilot_toggles.traffic_mode_jerk = [self.params.get_float("TrafficJerk"), self.frogpilot_toggles.aggressive_jerk] if self.frogpilot_toggles.custom_personalities else [1.0, 0.5]
    self.frogpilot_toggles.traffic_mode_t_follow = [self.params.get_float("TrafficFollow"), self.frogpilot_toggles.aggressive_follow] if self.frogpilot_toggles.custom_personalities else [0.5, 1.0]

    custom_theme = self.params.get_bool("CustomTheme")
    custom_sounds = self.params.get_int("CustomSounds") if custom_theme else 0
    frog_sounds = custom_sounds == 1
    self.frogpilot_toggles.goat_scream = frog_sounds and self.params.get_bool("GoatScream")
    self.frogpilot_toggles.holiday_themes = custom_theme and self.params.get_bool("HolidayThemes")
    self.frogpilot_toggles.random_events = custom_theme and self.params.get_bool("RandomEvents")

    custom_ui = self.params.get_bool("CustomUI")
    self.frogpilot_toggles.adjacent_lanes = custom_ui and self.params.get_bool("AdjacentPath")
    self.frogpilot_toggles.blind_spot_path = custom_ui and self.params.get_bool("BlindSpotPath")

    device_management = self.params.get_bool("DeviceManagement")
    self.frogpilot_toggles.increase_thermal_limits = device_management and self.params.get_bool("IncreaseThermalLimits")

    self.frogpilot_toggles.experimental_mode_via_press = self.params.get_bool("ExperimentalModeActivation")
    self.frogpilot_toggles.experimental_mode_via_distance = self.frogpilot_toggles.experimental_mode_via_press and self.params.get_bool("ExperimentalModeViaDistance")
    self.frogpilot_toggles.experimental_mode_via_lkas = self.frogpilot_toggles.experimental_mode_via_press and self.params.get_bool("ExperimentalModeViaLKAS")

    self.frogpilot_toggles.nudgeless = self.params.get_bool("NudgelessLaneChange")
    self.frogpilot_toggles.lane_change_delay = self.params.get_int("LaneChangeTime") if self.frogpilot_toggles.nudgeless else 0
    self.frogpilot_toggles.lane_detection = self.frogpilot_toggles.nudgeless and self.params.get_int("LaneDetectionWidth") != 0
    self.frogpilot_toggles.lane_detection_width = self.params.get_int("LaneDetectionWidth") * (1 if self.frogpilot_toggles.is_metric else CV.FOOT_TO_METER) / 10 if self.frogpilot_toggles.lane_detection else 0
    self.frogpilot_toggles.one_lane_change = self.frogpilot_toggles.nudgeless and self.params.get_bool("OneLaneChange")

    lateral_tune = self.params.get_bool("LateralTune")
    self.frogpilot_toggles.force_auto_tune = lateral_tune and self.params.get_float("ForceAutoTune")
    stock_steer_ratio = self.params.get_float("SteerRatioStock")
    self.frogpilot_toggles.steer_ratio = self.params.get_float("SteerRatio") if lateral_tune else stock_steer_ratio
    self.frogpilot_toggles.taco_tune = lateral_tune and self.params.get_bool("TacoTune")
    self.frogpilot_toggles.turn_desires = lateral_tune and self.params.get_bool("TurnDesires")
    self.frogpilot_toggles.use_custom_steer_ratio = self.frogpilot_toggles.steer_ratio != stock_steer_ratio

    self.frogpilot_toggles.long_pitch = self.params.get_bool("LongPitch")

    longitudinal_tune = self.CP.openpilotLongitudinalControl and self.params.get_bool("LongitudinalTune")
    self.frogpilot_toggles.acceleration_profile = self.params.get_int("AccelerationProfile") if longitudinal_tune else 0
    self.frogpilot_toggles.aggressive_acceleration = longitudinal_tune and self.params.get_bool("AggressiveAcceleration")
    self.frogpilot_toggles.deceleration_profile = self.params.get_int("DecelerationProfile") if longitudinal_tune else 0
    self.frogpilot_toggles.increased_stopping_distance = self.params.get_int("StoppingDistance") * (1 if self.frogpilot_toggles.is_metric else CV.FOOT_TO_METER) if longitudinal_tune else 0
    self.frogpilot_toggles.lead_detection_threshold = self.params.get_int("LeadDetectionThreshold") / 100 if longitudinal_tune else 0.5
    self.frogpilot_toggles.smoother_braking = longitudinal_tune and self.params.get_bool("SmoothBraking")
    self.frogpilot_toggles.smoother_braking_far_lead = self.frogpilot_toggles.smoother_braking and self.params.get_bool("SmoothBrakingFarLead")
    self.frogpilot_toggles.smoother_braking_jerk = self.frogpilot_toggles.smoother_braking and self.params.get_bool("SmoothBrakingJerk")
    self.frogpilot_toggles.sport_plus = longitudinal_tune and self.params.get_int("AccelerationProfile") == 3
    self.frogpilot_toggles.traffic_mode = longitudinal_tune and self.params.get_bool("TrafficMode")

    quality_of_life = self.params.get_bool("QOLControls")
    self.frogpilot_toggles.custom_cruise_increase = self.params.get_int("CustomCruise") if quality_of_life else 1
    self.frogpilot_toggles.custom_cruise_increase_long = self.params.get_int("CustomCruiseLong") if quality_of_life else 5
    self.frogpilot_toggles.pause_lateral_below_speed = self.params.get_int("PauseLateralSpeed") * (CV.KPH_TO_MS if self.frogpilot_toggles.is_metric else CV.MPH_TO_MS) if quality_of_life else 0
    self.frogpilot_toggles.pause_lateral_below_signal = quality_of_life and self.params.get_bool("PauseLateralOnSignal")
    self.frogpilot_toggles.reverse_cruise_increase = quality_of_life and self.params.get_bool("ReverseCruise")
    self.frogpilot_toggles.set_speed_offset = self.params.get_int("SetSpeedOffset") * (1 if self.frogpilot_toggles.is_metric else CV.MPH_TO_KPH) if quality_of_life else 0

    self.frogpilot_toggles.map_turn_speed_controller = self.CP.openpilotLongitudinalControl and self.params.get_bool("MTSCEnabled")
    self.frogpilot_toggles.mtsc_curvature_check = self.frogpilot_toggles.map_turn_speed_controller and self.params.get_bool("MTSCCurvatureCheck")
    self.params_memory.put_float("MapTargetLatA", 2 * (self.params.get_int("MTSCAggressiveness") / 100))

    self.frogpilot_toggles.sng_hack = self.params.get_bool("SNGHack")

    self.frogpilot_toggles.speed_limit_controller = self.CP.openpilotLongitudinalControl and self.params.get_bool("SpeedLimitController")
    self.frogpilot_toggles.force_mph_dashboard = self.frogpilot_toggles.speed_limit_controller and self.params.get_bool("ForceMPHDashboard")
    self.frogpilot_toggles.map_speed_lookahead_higher = self.params.get_int("SLCLookaheadHigher") if self.frogpilot_toggles.speed_limit_controller else 0
    self.frogpilot_toggles.map_speed_lookahead_lower = self.params.get_int("SLCLookaheadLower") if self.frogpilot_toggles.speed_limit_controller else 0
    self.frogpilot_toggles.offset1 = self.params.get_int("Offset1") if self.frogpilot_toggles.speed_limit_controller else 0
    self.frogpilot_toggles.offset2 = self.params.get_int("Offset2") if self.frogpilot_toggles.speed_limit_controller else 0
    self.frogpilot_toggles.offset3 = self.params.get_int("Offset3") if self.frogpilot_toggles.speed_limit_controller else 0
    self.frogpilot_toggles.offset4 = self.params.get_int("Offset4") if self.frogpilot_toggles.speed_limit_controller else 0
    self.frogpilot_toggles.set_speed_limit = self.frogpilot_toggles.speed_limit_controller and self.params.get_bool("SetSpeedLimit")
    self.frogpilot_toggles.speed_limit_alert = self.frogpilot_toggles.speed_limit_controller and self.params.get_bool("SpeedLimitChangedAlert")
    self.frogpilot_toggles.speed_limit_confirmation = self.frogpilot_toggles.speed_limit_controller and self.params.get_bool("SLCConfirmation")
    self.frogpilot_toggles.speed_limit_confirmation_lower = self.frogpilot_toggles.speed_limit_confirmation and self.params.get_bool("SLCConfirmationLower")
    self.frogpilot_toggles.speed_limit_confirmation_higher = self.frogpilot_toggles.speed_limit_confirmation and self.params.get_bool("SLCConfirmationHigher")
    self.frogpilot_toggles.speed_limit_controller_override = self.params.get_int("SLCOverride") if self.frogpilot_toggles.speed_limit_controller else 0
    self.frogpilot_toggles.speed_limit_controller_fallback = self.params.get_int("SLCFallback") if self.frogpilot_toggles.speed_limit_controller else 0
    self.frogpilot_toggles.speed_limit_priority1 = self.params.get("SLCPriority1", encoding='utf-8') if self.frogpilot_toggles.speed_limit_controller else None
    self.frogpilot_toggles.speed_limit_priority2 = self.params.get("SLCPriority2", encoding='utf-8') if self.frogpilot_toggles.speed_limit_controller else None
    self.frogpilot_toggles.speed_limit_priority3 = self.params.get("SLCPriority3", encoding='utf-8') if self.frogpilot_toggles.speed_limit_controller else None
    self.frogpilot_toggles.speed_limit_priority_highest = self.frogpilot_toggles.speed_limit_priority1 == "Highest"
    self.frogpilot_toggles.speed_limit_priority_lowest = self.frogpilot_toggles.speed_limit_priority1 == "Lowest"

    toyota_doors = self.params.get_bool("ToyotaDoors")
    self.frogpilot_toggles.lock_doors = toyota_doors and self.params.get_bool("LockDoors")
    self.frogpilot_toggles.unlock_doors = toyota_doors and self.params.get_bool("UnlockDoors")

    self.frogpilot_toggles.use_ev_tables = self.params.get_bool("EVTable")

    self.frogpilot_toggles.vision_turn_controller = self.CP.openpilotLongitudinalControl and self.params.get_bool("VisionTurnControl")
    self.frogpilot_toggles.curve_sensitivity = self.params.get_int("CurveSensitivity") / 100 if self.frogpilot_toggles.vision_turn_controller else 1
    self.frogpilot_toggles.turn_aggressiveness = self.params.get_int("TurnAggressiveness") / 100 if self.frogpilot_toggles.vision_turn_controller else 1

FrogPilotVariables = FrogPilotVariables()
FrogPilotToggles = FrogPilotVariables.frogpilot_toggles
