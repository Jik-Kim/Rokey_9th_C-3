import omni.usd

VELOCITY = 0.5

stage = omni.usd.get_context().get_stage()
count = 0
for prim in stage.Traverse():
    if prim.GetTypeName() == "OmniGraph":
        attr = prim.GetAttribute("graph:variable:Velocity")
        if attr:
            old = attr.Get()
            attr.Set(VELOCITY)
            print(f"  {old} -> {VELOCITY}   {prim.GetPath()}")
            count += 1
print(f"\n총 {count}개 ConveyorBeltGraph 에 Velocity={VELOCITY} 적용")
