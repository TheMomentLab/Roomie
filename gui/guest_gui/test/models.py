# models.py (수정 후)

from pydantic import BaseModel, Field
from typing import List, Optional

# --- 공통 모델 ---
class OrderItem(BaseModel):
    name: str
    quantity: int
    price: int

class OrderDetails(BaseModel):
    items: List[OrderItem]

# --- 신규 추가 모델 ---
class RobotStatus(BaseModel):
    x: float
    y: float
    floor_id: int = Field(alias="floorId") # JSON의 floorId와 매핑

# --- 페이로드 모델 (명세서에 맞춰 필드 수정 및 추가) ---
class RequestPayload(BaseModel):
    # create_call_task 용
    location: Optional[str] = None
    task_type: Optional[int] = None

    # 메뉴, 배송, 내역 조회 등에서 사용
    location_name: Optional[str] = None
    request_location: Optional[str] = None
    task_type_name: Optional[str] = None
    task_name: Optional[str] = None
    order_details: Optional[OrderDetails] = None

# --- 최상위 요청 모델 ---
class RequestModel(BaseModel):
    type: str # "request"
    action: str
    payload: Optional[RequestPayload] = {}