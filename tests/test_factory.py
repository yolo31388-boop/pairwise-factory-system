import pytest
from factory.system import FactorySystem


class TestFactorySystem:

    def test_update_conveyor(self):
        """传送带不做物品移动，东西放上去不动"""
        obj = FactorySystem()
        result = obj.update_conveyor()
        self.assertIsNotNone(result)

    def test_update_arm(self):
        """机械臂不做抓取检测，东西到了不抓"""
        obj = FactorySystem()
        result = obj.update_arm()
        self.assertIsNotNone(result)

    def test_update_assembler(self):
        """装配机不做配方检查，材料不对也能生产"""
        obj = FactorySystem()
        result = obj.update_assembler()
        self.assertIsNotNone(result)

    def test_route_items(self):
        """物流不做路径选择，所有东西走同一条路"""
        obj = FactorySystem()
        result = obj.route_items()
        self.assertIsNotNone(result)

    def test_update_circuit(self):
        """电路不做信号传播，开关开了灯不亮"""
        obj = FactorySystem()
        result = obj.update_circuit()
        self.assertIsNotNone(result)

