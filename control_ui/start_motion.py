"""Compatibility response for the retired training-boundary wrist recovery."""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field, ConfigDict


class StartOnce(BaseModel):
    model_config = ConfigDict(extra='forbid')
    reference_sha256: str = Field(pattern=r'^[a-f0-9]{64}$')
    expected_joint_rad: list[float] = Field(min_length=6,max_length=6)
    camera_request_id: str = Field(pattern=r'^[a-f0-9]{32}$')


def install(app, service_provider):
    router=APIRouter(prefix='/api/bench/motion')

    @router.get('/capabilities')
    def capabilities():
        return {'high_wrist_recovery':False,'retired':True,'replacement':'/coordinates',
                'joint_limit_profile':'roarm_m3_vendor_nominal_v1'}

    @router.post('/start-once')
    def start_once(req:StartOnce):
        raise HTTPException(status_code=409,detail='旧的腕部收回流程已停用：±60°训练限制已取消。请在/coordinates预览并执行所需XYZ，无需先收回腕部。')

    app.include_router(router)
