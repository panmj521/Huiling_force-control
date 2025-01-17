import math

def inverse_kinematic(x,y,z,r,L1 = 325,L2 = 175):

    #z轴方向完全由第一个平移关节控制
    thetha1 = z
    #计算末端执行器在xy平面上的位置
    r_xy_square = x**2 + y**2
    r_xy = math.sqrt(r_xy_square)

    #计算第二个关节和第三个关节的角度
    cos_thetha3 = (r_xy_square-L1**2-L2**2) / (2*L1*L2)
    if cos_thetha3 >1 or cos_thetha3 < -1:
        raise ValueError("目标位置不可达")
    sin_thetha3 = math.sqrt(1-cos_thetha3**2)
    #第三个关节角度，可正可负
    thetha3 = math.atan2(sin_thetha3,cos_thetha3)
    
    ##计算第二个关节角度，通过建立β和φ之间的关系获得第二个关节角度
    β = math.atan2(y,x)
    cos_φ = (r_xy_square + L1**2 - L2**2) / (2*L1*r_xy)
    φ = math.acos(cos_φ)
    if thetha3 < 0:
        theth2 = β + φ
    else:
        theth2 = β - φ
    #计算第四个关节角度,为四关节绕Z轴旋转的角度，只由第四关节控制
    thetha4 = r

    return thetha1, theth2, thetha3, thetha4


if __name__ == "__main__":
    thetha1, theth2, thetha3, thetha4 = inverse_kinematic()


    