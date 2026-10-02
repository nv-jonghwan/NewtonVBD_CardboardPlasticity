"""Rate-independent hinge return mapping, operator-split after Newton VBD.

Internal moments are conjugate to dihedral angle (N m); the material parameter
is yield curvature (1/m). This preserves the threshold when mesh width changes.
"""
import warp as wp
import newton

@wp.func
def dihedral(x0:wp.vec3,x1:wp.vec3,x2:wp.vec3,x3:wp.vec3):
    n0=wp.normalize(wp.cross(x2-x0,x3-x0))
    n1=wp.normalize(wp.cross(x3-x1,x2-x1))
    edge=wp.normalize(x3-x2)
    return wp.atan2(wp.dot(wp.cross(n0,n1),edge),wp.dot(n0,n1))

@wp.kernel
def return_map(q:wp.array[wp.vec3],edges:wp.array2d[int],length:wp.array[float],reference:wp.array[float],dual:wp.array[float],base_ke:wp.array[float],
               yield_curvature:float,hardening_ratio:float,damage_rate:float,residual:float,enabled:int,relaxation_time:float,
               old_p:wp.array[float],old_a:wp.array[float],old_w:wp.array[float],
               new_p:wp.array[float],new_a:wp.array[float],new_w:wp.array[float],new_d:wp.array[float],
               rest_angle:wp.array[float],properties:wp.array2d[float]):
    i=wp.tid();p=old_p[i];a=old_a[i];work=old_w[i]
    if edges[i,0]>=0 and edges[i,1]>=0 and enabled==1:
        theta=dihedral(q[edges[i,0]],q[edges[i,1]],q[edges[i,2]],q[edges[i,3]])
        # Match Newton's unwrapped quadratic bending convention; intended folds remain away from the atan2 branch cut.
        elastic=theta-reference[i]-p
        damage=wp.min(1.0-residual,1.0-wp.exp(-damage_rate*a))
        k0=base_ke[i]*length[i];k=k0*(1.0-damage)
        y=k0*yield_curvature*dual[i]
        H=hardening_ratio*k0
        trial=k*elastic;f=wp.abs(trial)-(y+H*a)
        if f>0.0:
            dg=f/(k+H)
            p=p+wp.sign(trial)*dg
            # Dissipation = yield moment * plastic-angle increment;
            # isotropic hardening is stored energy, not dissipated energy.
            work=work+y*dg
            a=a+dg
    damage=wp.min(1.0-residual,1.0-wp.exp(-damage_rate*a))
    new_p[i]=p;new_a[i]=a;new_w[i]=work;new_d[i]=damage
    rest_angle[i]=reference[i]+p
    properties[i,0]=base_ke[i]*(1.0-damage)
    if relaxation_time>0.0:properties[i,1]=relaxation_time*properties[i,0]

def register_state(builder):
    for name in ['plastic_angle','accumulated_angle','plastic_work','damage']:
        builder.add_custom_attribute(newton.ModelBuilder.CustomAttribute(name=name,namespace='cardboard',dtype=wp.float32,frequency=newton.Model.AttributeFrequency.EDGE,assignment=newton.Model.AttributeAssignment.STATE,default=0.0))


@wp.kernel
def return_map_crease(q:wp.array[wp.vec3],edges:wp.array2d[int],length:wp.array[float],reference:wp.array[float],dual:wp.array[float],base_ke:wp.array[float],
                      yield_curvature:float,hardening_ratio:float,damage_rate:float,residual:float,enabled:int,relaxation_time:float,
                      old_p:wp.array[float],old_a:wp.array[float],old_w:wp.array[float],
                      new_p:wp.array[float],new_a:wp.array[float],new_w:wp.array[float],new_d:wp.array[float],
                      rest_angle:wp.array[float],properties:wp.array2d[float],damage_length:float):
    """Optional local crease damage, with a material length rather than mesh angle.

    Both unloading stiffness and yield moment degrade. Intact elastic stiffness
    is unchanged. This local law is not nonlocal/fracture-energy regularization.
    """
    i=wp.tid();p=old_p[i];a=old_a[i];work=old_w[i]
    d0=wp.min(1.0-residual,1.0-wp.exp(-damage_rate*a*damage_length/dual[i]))
    damage=d0
    if edges[i,0]>=0 and edges[i,1]>=0 and enabled==1:
        theta=dihedral(q[edges[i,0]],q[edges[i,1]],q[edges[i,2]],q[edges[i,3]])
        k0=base_ke[i]*length[i];H0=hardening_ratio*k0
        k=k0*(1.0-d0);H=H0*(1.0-d0)
        y=k0*yield_curvature*dual[i]*(1.0-d0)
        trial=k*(theta-reference[i]-p)
        dg=wp.max(0.0,(wp.abs(trial)-y-H*a)/(k+H))
        p+=wp.sign(trial)*dg;a+=dg
        damage=wp.min(1.0-residual,1.0-wp.exp(-damage_rate*a*damage_length/dual[i]))
        elastic=theta-reference[i]-p
        # Plastic dissipation plus released elastic/hardening energy from damage.
        work+=y*dg+0.5*(damage-d0)*(k0*elastic*elastic+H0*a*a)
    new_p[i]=p;new_a[i]=a;new_w[i]=work;new_d[i]=damage
    rest_angle[i]=reference[i]+p
    properties[i,0]=base_ke[i]*(1.0-damage)
    if relaxation_time>0.0:properties[i,1]=relaxation_time*properties[i,0]

def project_scalar(theta,reference,p,alpha,k,y,H):
    """Independent scalar reference for constitutive regression tests."""
    import math
    trial=k*(theta-reference-p);dg=max(0.,(abs(trial)-y-H*alpha)/(k+H))
    return p+math.copysign(dg,trial),alpha+dg,y*dg
