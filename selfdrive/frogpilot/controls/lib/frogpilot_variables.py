from types import SimpleNamespace

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params

from openpilot.selfdrive.frogpilot.controls.lib.model_manager import RADARLESS_MODELS

class FrogPilotVariables:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")

    self.toggles = SimpleNamespace()

    with car.CarParams.from_bytes(self.params.get("CarParams", block=True)) as msg:
      self.CP = msg

    self.toggles.radarless_model = self.params.get("Model", block=True, encoding='utf-8') in RADARLESS_MODELS

    self.update_frogpilot_params()

  def update_frogpilot_params(self):
    self.toggles.is_metric = self.params.get_bool("IsMetric")

    always_on_lateral = self.params.get_bool("AlwaysOnLateral")
    self.toggles.always_on_lateral_pause_speed = always_on_lateral and self.params.get_int("PauseAOLOnBrake")

    self.toggles.conditional_experimental_mode = self.CP.openpilotLongitudinalControl and self.params.get_bool("ConditionalExperimental")
    self.toggles.conditional_curves = self.toggles.conditional_experimental_mode and self.params.get_bool("CECurves")
    self.toggles.conditional_curves_lead = self.toggles.conditional_curves and self.params.get_bool("CECurvesLead")
    self.toggles.conditional_limit = self.params.get_int("CESpeed") * (CV.KPH_TO_MS if self.toggles.is_metric else CV.MPH_TO_MS) if self.toggles.conditional_experimental_mode else 0
    self.toggles.conditional_limit_lead = self.params.get_int("CESpeedLead") * (CV.KPH_TO_MS if self.toggles.is_metric else CV.MPH_TO_MS) if self.toggles.conditional_experimental_mode else 0
    self.toggles.conditional_navigation = self.toggles.conditional_experimental_mode and self.params.get_bool("CENavigation")
    self.toggles.conditional_navigation_intersections = self.toggles.conditional_navigation and self.params.get_bool("CENavigationIntersections")
    self.toggles.conditional_navigation_lead = self.toggles.conditional_navigation and self.params.get_bool("CENavigationLead")
    self.toggles.conditional_navigation_turns = self.toggles.conditional_navigation and self.params.get_bool("CENavigationTurns")
    self.toggles.conditional_signal = self.toggles.conditional_experimental_mode and self.params.get_bool("CESignal")
    self.toggles.conditional_slower_lead = self.toggles.conditional_experimental_mode and self.params.get_bool("CESlowerLead")
    self.toggles.conditional_stop_lights = self.toggles.conditional_experimental_mode and self.params.get_bool("CEStopLights")
    self.toggles.conditional_stop_lights_lead = self.toggles.conditional_stop_lights and self.params.get_bool("CEStopLightsLead")

    custom_alerts = self.params.get_bool("CustomAlerts")
    self.toggles.green_light_alert = custom_alerts and self.params.get_bool("GreenLightAlert")
    self.toggles.lead_departing_alert = not self.toggles.radarless_model and custom_alerts and self.params.get_bool("LeadDepartingAlert")
    self.toggles.loud_blindspot_alert = custom_alerts and self.params.get_bool("LoudBlindspotAlert")

    self.toggles.custom_personalities = self.params.get_bool("CustomPersonalities")
    self.toggles.aggressive_jerk = self.params.get_float("AggressiveJerk")
    self.toggles.aggressive_follow = self.params.get_float("AggressiveFollow")
    self.toggles.standard_jerk = self.params.get_float("StandardJerk")
    self.toggles.standard_follow = self.params.get_float("StandardFollow")
    self.toggles.relaxed_jerk = self.params.get_float("RelaxedJerk")
    self.toggles.relaxed_follow = self.params.get_float("RelaxedFollow")
    self.toggles.traffic_mode_jerk = [self.params.get_float("TrafficJerk"), self.toggles.aggressive_jerk] if self.toggles.custom_personalities else [1.0, 0.5]
    self.toggles.traffic_mode_t_follow = [self.params.get_float("TrafficFollow"), self.toggles.aggressive_follow] if self.toggles.custom_personalities else [0.5, 1.0]

    custom_theme = self.params.get_bool("CustomTheme")
    custom_sounds = self.params.get_int("CustomSounds") if custom_theme else 0
    frog_sounds = custom_sounds == 1
    self.toggles.goat_scream = frog_sounds and self.params.get_bool("GoatScream")
    self.toggles.holiday_themes = custom_theme and self.params.get_bool("HolidayThemes")
    self.toggles.random_events = custom_theme and self.params.get_bool("RandomEvents")

    custom_ui = self.params.get_bool("CustomUI")
    self.toggles.adjacent_lanes = custom_ui and self.params.get_bool("AdjacentPath")
    self.toggles.blind_spot_path = custom_ui and self.params.get_bool("BlindSpotPath")

    device_management = self.params.get_bool("DeviceManagement")
    self.toggles.increase_thermal_limits = device_management and self.params.get_bool("IncreaseThermalLimits")

    self.toggles.experimental_mode_via_press = self.params.get_bool("ExperimentalModeActivation")
    self.toggles.experimental_mode_via_distance = self.toggles.experimental_mode_via_press and self.params.get_bool("ExperimentalModeViaDistance")
    self.toggles.experimental_mode_via_lkas = self.toggles.experimental_mode_via_press and self.params.get_bool("ExperimentalModeViaLKAS")

    self.toggles.nudgeless = self.params.get_bool("NudgelessLaneChange")
    self.toggles.lane_change_delay = self.params.get_int("LaneChangeTime") if self.toggles.nudgeless else 0
    self.toggles.lane_detection = self.toggles.nudgeless and self.params.get_int("LaneDetectionWidth") != 0
    self.toggles.lane_detection_width = self.params.get_int("LaneDetectionWidth") * (1 if self.toggles.is_metric else CV.FOOT_TO_METER) / 10 if self.toggles.lane_detection else 0
    self.toggles.one_lane_change = self.toggles.nudgeless and self.params.get_bool("OneLaneChange")

    lateral_tune = self.params.get_bool("LateralTune")
    self.toggles.force_auto_tune = lateral_tune and self.params.get_float("ForceAutoTune")
    stock_steer_ratio = self.params.get_float("SteerRatioStock")
    self.toggles.steer_ratio = self.params.get_float("SteerRatio") if lateral_tune else stock_steer_ratio
    self.toggles.taco_tune = lateral_tune and self.params.get_bool("TacoTune")
    self.toggles.turn_desires = lateral_tune and self.params.get_bool("TurnDesires")
    self.toggles.use_custom_steer_ratio = self.toggles.steer_ratio != stock_steer_ratio

    self.toggles.long_pitch = self.params.get_bool("LongPitch")

    longitudinal_tune = self.CP.openpilotLongitudinalControl and self.params.get_bool("LongitudinalTune")
    self.toggles.acceleration_profile = self.params.get_int("AccelerationProfile") if longitudinal_tune else 0
    self.toggles.aggressive_acceleration = longitudinal_tune and self.params.get_bool("AggressiveAcceleration")
    self.toggles.deceleration_profile = self.params.get_int("DecelerationProfile") if longitudinal_tune else 0
    self.toggles.increased_stopping_distance = self.params.get_int("StoppingDistance") * (1 if self.toggles.is_metric else CV.FOOT_TO_METER) if longitudinal_tune else 0
    self.toggles.lead_detection_threshold = self.params.get_int("LeadDetectionThreshold") / 100 if longitudinal_tune else 0.5
    self.toggles.smoother_braking = longitudinal_tune and self.params.get_bool("SmoothBraking")
    self.toggles.smoother_braking_far_lead = self.toggles.smoother_braking and self.params.get_bool("SmoothBrakingFarLead")
    self.toggles.smoother_braking_jerk = self.toggles.smoother_braking and self.params.get_bool("SmoothBrakingJerk")
    self.toggles.sport_plus = longitudinal_tune and self.params.get_int("AccelerationProfile") == 3
    self.toggles.traffic_mode = longitudinal_tune and self.params.get_bool("TrafficMode")

    quality_of_life = self.params.get_bool("QOLControls")
    self.toggles.custom_cruise_increase = self.params.get_int("CustomCruise") if quality_of_life else 1
    self.toggles.custom_cruise_increase_long = self.params.get_int("CustomCruiseLong") if quality_of_life else 5
    self.toggles.pause_lateral_below_speed = self.params.get_int("PauseLateralSpeed") * (CV.KPH_TO_MS if self.toggles.is_metric else CV.MPH_TO_MS) if quality_of_life else 0
    self.toggles.pause_lateral_below_signal = quality_of_life and self.params.get_bool("PauseLateralOnSignal")
    self.toggles.reverse_cruise_increase = quality_of_life and self.params.get_bool("ReverseCruise")
    self.toggles.set_speed_offset = self.params.get_int("SetSpeedOffset") * (1 if self.toggles.is_metric else CV.MPH_TO_KPH) if quality_of_life else 0

    self.toggles.map_turn_speed_controller = self.CP.openpilotLongitudinalControl and self.params.get_bool("MTSCEnabled")
    self.toggles.mtsc_curvature_check = self.toggles.map_turn_speed_controller and self.params.get_bool("MTSCCurvatureCheck")
    self.params_memory.put_float("MapTargetLatA", 2 * (self.params.get_int("MTSCAggressiveness") / 100))

    self.toggles.sng_hack = self.params.get_bool("SNGHack")

    self.toggles.speed_limit_controller = self.CP.openpilotLongitudinalControl and self.params.get_bool("SpeedLimitController")
    self.toggles.force_mph_dashboard = self.toggles.speed_limit_controller and self.params.get_bool("ForceMPHDashboard")
    self.toggles.map_speed_lookahead_higher = self.params.get_int("SLCLookaheadHigher") if self.toggles.speed_limit_controller else 0
    self.toggles.map_speed_lookahead_lower = self.params.get_int("SLCLookaheadLower") if self.toggles.speed_limit_controller else 0
    self.toggles.offset1 = self.params.get_int("Offset1") if self.toggles.speed_limit_controller else 0
    self.toggles.offset2 = self.params.get_int("Offset2") if self.toggles.speed_limit_controller else 0
    self.toggles.offset3 = self.params.get_int("Offset3") if self.toggles.speed_limit_controller else 0
    self.toggles.offset4 = self.params.get_int("Offset4") if self.toggles.speed_limit_controller else 0
    self.toggles.set_speed_limit = self.toggles.speed_limit_controller and self.params.get_bool("SetSpeedLimit")
    self.toggles.speed_limit_alert = self.toggles.speed_limit_controller and self.params.get_bool("SpeedLimitChangedAlert")
    self.toggles.speed_limit_confirmation = self.toggles.speed_limit_controller and self.params.get_bool("SLCConfirmation")
    self.toggles.speed_limit_confirmation_lower = self.toggles.speed_limit_confirmation and self.params.get_bool("SLCConfirmationLower")
    self.toggles.speed_limit_confirmation_higher = self.toggles.speed_limit_confirmation and self.params.get_bool("SLCConfirmationHigher")
    self.toggles.speed_limit_controller_override = self.params.get_int("SLCOverride") if self.toggles.speed_limit_controller else 0
    self.toggles.speed_limit_controller_fallback = self.params.get_int("SLCFallback") if self.toggles.speed_limit_controller else 0
    self.toggles.speed_limit_priority1 = self.params.get("SLCPriority1", encoding='utf-8') if self.toggles.speed_limit_controller else None
    self.toggles.speed_limit_priority2 = self.params.get("SLCPriority2", encoding='utf-8') if self.toggles.speed_limit_controller else None
    self.toggles.speed_limit_priority3 = self.params.get("SLCPriority3", encoding='utf-8') if self.toggles.speed_limit_controller else None
    self.toggles.speed_limit_priority_highest = self.toggles.speed_limit_priority1 == "Highest"
    self.toggles.speed_limit_priority_lowest = self.toggles.speed_limit_priority1 == "Lowest"

    toyota_doors = self.params.get_bool("ToyotaDoors")
    self.toggles.lock_doors = toyota_doors and self.params.get_bool("LockDoors")
    self.toggles.unlock_doors = toyota_doors and self.params.get_bool("UnlockDoors")

    self.toggles.use_ev_tables = self.params.get_bool("EVTable")

    self.toggles.vision_turn_controller = self.CP.openpilotLongitudinalControl and self.params.get_bool("VisionTurnControl")
    self.toggles.curve_sensitivity = self.params.get_int("CurveSensitivity") / 100 if self.toggles.vision_turn_controller else 1
    self.toggles.turn_aggressiveness = self.params.get_int("TurnAggressiveness") / 100 if self.toggles.vision_turn_controller else 1

FrogPilotVariables = FrogPilotVariables()
FrogPilotToggles = FrogPilotVariables.toggles
