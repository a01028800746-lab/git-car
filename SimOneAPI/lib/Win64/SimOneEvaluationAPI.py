from ctypes import c_bool, c_char_p

from SimOneIOStruct import SimoneAPI


def SoInitEvaluationServiceWithLocalData(main_vehicle_id, with_main_vehicle=True):
    SimoneAPI.InitEvaluationServiceWithLocalData.argtypes = [c_char_p, c_bool]
    SimoneAPI.InitEvaluationServiceWithLocalData.restype = c_bool
    return SimoneAPI.InitEvaluationServiceWithLocalData(
        main_vehicle_id.encode("utf-8"), with_main_vehicle
    )


def SoSaveEvaluationRecord():
    SimoneAPI.SaveEvaluationRecord.argtypes = []
    SimoneAPI.SaveEvaluationRecord.restype = c_bool
    return SimoneAPI.SaveEvaluationRecord()
