'''
This program creates a track based on vectors for given points for the
KUKA KR10 R1420.

It uses a saved base in the controller

.dat and .src are the outputs with the given name.
'''
import time
import numpy as np
from scipy.io import savemat

named_tuple = time.localtime()
time_string = time.strftime("parcour-time%H_%M_%S-date%m_%d_%Y", named_tuple)

class Parcours:

    __trust = False # Blocks certain points or movements when False
    object_lims = None
    object_max = None
    security_factor = 1.2
    last_point_coord = None # For checking the path between last two points
    _points = [] # Every point's coordinate with [x,y,z,A,B,C]
    path_control_unit = r'KRC:\R1\Parcours\Nicolas F' # Path where the parcours 
    # will be saved in the robot controller

    def get_points(self):
        return self._points

    # Movement of the axes
    turn = int('110010',base=2) # 6-bit binary in int representation
    status = int('110',base=2) # 3-bit binary in int representation
    '''
    T is a 6-bit binary value (normally represented as an integer), with bit 0 
    representing Axis 1, and bit 5 representing Axis 6, and so on in between. 
    The bit is set to 0 if the axis is >= 0deg, and 1 if the axis is <0deg.

    S is a 3-bit binary value. Bit 0 is False only if the intersection of the 
    wrist axes (center of A5, essentially) is "forward" of A1 (that is, 
    imagine the YZ plane of RobRoot rotating with A1 -- as long as the position
    of the wrist center has a positive X value in this rotating coordinate
    frame), and is True otherwise. Bit 1 is False if A3 is less than a certain
    angle (which varies depending on your model, you may have to determine this
    experimentally -- the angle is 0deg if your A3 and A4 are co-planar), 
    and True otherwise. Bit 2 is False for ((0<=A5<180) OR (A5 < -180)) 
    and True for ((-180<=A5<0) OR (A5 >=180))
    '''

    def set_turn_status(self,param:dict):
        self.turn = param['t']
        self.status = param['s']

    def __update_last_point(self,coord):
        self.last_point_coord = coord

    def __init__(self,name=time_string,tool = '[4]:Microflown3D', velocity=50,
                 base='[0]', base_coord=np.zeros(3)):
        # Initiate .dat and .src files as list of strings

        assert len(base_coord) == 3, 'The base coordinates must be a vector' +\
                               'with [x, y, z]. Rotations must be expressed' +\
                               'in the robot`s basis.'

        self.name = name
        self.__counter_points = 1
        self.tool = tool
        self.tool_no = int(tool[1])

        # Base handling
        self.base = base
        self.base_no = int(self.base[1])
        self.base_coord = base_coord

        self.dat_file = ['&ACCESS RVP',
        '&REL insertPointNumber',
        '&PARAM EDITMASK = *',
        r'&PARAM TEMPLATE = C:\KRC\Roboter\Template\vorgabe',
        '&PARAM DISKPATH = '+self.path_control_unit,
        f'DEFDAT {name}',
        r';FOLD EXTERNAL DECLARATIONS;%{PE}%MKUKATPBASIS,%CEXT,%VCOMMON,%P',
        r';FOLD BASISTECH EXT;%{PE}%MKUKATPBASIS,%CEXT,%VEXT,%P',
        'EXT  BAS (BAS_COMMAND  :IN,REAL  :IN )',
        'DECL INT SUCCESS',
        ';ENDFOLD (BASISTECH EXT)',
        r';FOLD USER EXT;%{E}%MKUKATPUSER,%CEXT,%VEXT,%P',
        ';Make your modifications here \n',

        ';ENDFOLD (USER EXT)',
        ';ENDFOLD (EXTERNAL DECLARATIONS)\n']

        self.src_file = [r'&ACCESS RVP',
        '&REL insertPointNumber',
        '&PARAM EDITMASK = *',
        r'&PARAM TEMPLATE = C:\KRC\Roboter\Template\vorgabe',
        '&PARAM DISKPATH = '+self.path_control_unit,
        f'DEF {name}( )',
        r';FOLD INI;%{PE}',
        '  ;FOLD BASISTECH INI',
        r'    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )',
        r'    INTERRUPT ON 3 ',
        '    BAS (#INITMOV,0 )',
        '  ;ENDFOLD (BASISTECH INI)',
        '  ;FOLD USER INI',
        '    ;Make your modifications here \n',

        '  ;ENDFOLD (USER INI)',
        ';ENDFOLD (INI)\n',

        r';FOLD SPTP HOME Vel=100 % DEFAULT ;%{PE}'.replace('=100',f'={velocity}'),
        r';FOLD Parameters ;%{h}',

        self.__join(''';Params IlfProvider=kukaroboter.basistech.inlineforms.movement.spline; 
        Kuka.IsGlobalPoint=False; Kuka.PointName=HOME; Kuka.BlendingEnabled=False; 
        Kuka.MoveDataPtpName=DEFAULT; Kuka.VelocityPtp=100; 
        Kuka.VelocityFieldEnabled=True; Kuka.CurrentCDSetIndex=0; 
        Kuka.MovementParameterFieldEnabled=True; 
        IlfCommand=SPTP'''),
        ';ENDFOLD',
        self.__join('''SPTP XHOME WITH $VEL_AXIS[1] = SVEL_JOINT(100.0), 
        $TOOL = STOOL2(FHOME), $BASE = SBASE(FHOME.BASE_NO), 
        $IPO_MODE = SIPO_MODE(FHOME.IPO_FRAME), 
        $LOAD = SLOAD(FHOME.TOOL_NO), 
        $ACC_AXIS[1] = SACC_JOINT(PDEFAULT), 
        $APO = SAPO_PTP(PDEFAULT), 
        $GEAR_JERK[1] = SGEAR_JERK(PDEFAULT), 
        $COLLMON_TOL_PRO[1] = USE_CM_PRO_VALUES(0)'''),
        ';ENDFOLD\n']

    def __join(self,st:str,curly=True):
        '''Helper Function to replace curly braces and join all lines in a
        given string.'''
        if curly:
            st = st.replace('(((','{').replace(')))','}')
        return st.replace('\n','').replace('  ','')

    def add_point_SPTP(self, coord, rot, velocity=50, param:dict=None,
                       marker=1, coord_trafo=True):
        '''Adds a new SPTP point to track using coordinates, rotation, velocity
        and, if given, turn and status parameters.

        Parameters
        ------------------
        coord: array with length 3 containing (x,y,z) coordinates.
        rotation: array with length 3 containing (A,B,C) rotations.
        
        velocity: (optional) int with velocity in %.
        param: (optional) dictionary with keys 't' for Turn and 's' for Status.
        If no dictionary is given, it takes the default values or the last given.'''

        if coord_trafo:#  Basis change to robot's universal
            for ind in range(3):
                coord[ind] += self.base_coord[ind]

        if param is not None:
            assert len(param['t'])==6 and type(param['t'])==str, \
                'Turn must be a 6-bit binary saved as str'
            assert len(param['s'])==3 and type(param['s'])==str, \
                'Status must be a 3-bit binary saved as str'
            self.turn = int(param['t'],base=2)
            self.status = int(param['s'],base=2)

        if self.object_lims is not None:
            self._check_object_lims(coord)
            if self.last_point_coord is not None and not self._check_path(coord):
                print(f'''Warning! The linear path crosses the object limits.
                Point no.: {self.__counter_points}''')
                if not self.__trust:
                    assert(self._check_path(coord)),\
                        'Turn trust mode on for defining such paths.'

        assert velocity > 0 and velocity <= 100,\
            'Velocity for SPTP cannot be out 0-100%'
        assert len(coord)==3,\
            'Coordinates must be an array with ordered XYZ-Coordinates'
        assert len(rot)==3,\
            'Rotation must be an array with ABC-Rotations data'

        ## SRC-File
        newPointSRC = [self.__join(f''';FOLD SPTP P{self.__counter_points} Vel=
        {velocity} % PDAT{self.__counter_points} Tool{self.tool} Base{self.base}  
          ;%(((PE)))'''),
        r';FOLD Parameters ;%{h}',
        self.__join(f''';Params 
         IlfProvider=kukaroboter.basistech.inlineforms.movement.spline; 
        Kuka.IsGlobalPoint=False; 
        Kuka.PointName=P{self.__counter_points}; 
        Kuka.BlendingEnabled=False; 
        Kuka.MoveDataPtpName=PDAT{self.__counter_points}; 
        Kuka.VelocityPtp={velocity}; 
        Kuka.VelocityFieldEnabled=True; 
        Kuka.ColDetectFieldEnabled=True; 
        Kuka.CurrentCDSetIndex=0; 
        Kuka.MovementParameterFieldEnabled=True; 
        IlfCommand=SPTP'''),
        ';ENDFOLD',
        self.__join(f'''SPTP XP{self.__counter_points} WITH 
        $VEL_AXIS[1] = SVEL_JOINT({velocity}.0), 
        $TOOL = STOOL2(FP{self.__counter_points}), 
        $BASE = SBASE(FP{self.__counter_points}.BASE_NO), 
        $IPO_MODE = SIPO_MODE(FP{self.__counter_points}.IPO_FRAME), 
        $LOAD = SLOAD(FP{self.__counter_points}.TOOL_NO), 
        $ACC_AXIS[1] = SACC_JOINT(PPDAT{self.__counter_points}), 
        $APO = SAPO_PTP(PPDAT{self.__counter_points}), 
        $GEAR_JERK[1] = SGEAR_JERK(PPDAT{self.__counter_points}), 
        $COLLMON_TOL_PRO[1] = USE_CM_PRO_VALUES(0)'''),
        ';ENDFOLD\n']

        ## DAT-File
        newPointDAT = [self.__join(f'''DECL E6POS 
        XP{self.__counter_points}=(((X {coord[0]},Y {coord[1]},Z {coord[2]},
        A {rot[0]},B {rot[1]},C {rot[2]},S {self.status},T {self.turn},
        E1 0.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0)))'''),
        self.__join(f'''DECL FDAT FP{self.__counter_points}
        =(((TOOL_NO {self.tool_no},BASE_NO {self.base_no},
        IPO_FRAME #BASE,POINT2[] " ")))'''),
        self.__join(f'''DECL PDAT 
        PPDAT{self.__counter_points}=(((VEL {velocity}.000,
        ACC {velocity}.000,APO_DIST 500.000,
        APO_MODE #CDIS,GEAR_JERK 100.000,EXAX_IGN 0)))'''),'']

        if self.__counter_points == 1 :
            self.dat_file.append(self.__join(f'''DECL MODULEPARAM_T LAST_TP_PARAMS=(((PARAMS[] 
            "Kuka.VelocityFieldEnabled=False; 
            Kuka.ColDetectFieldEnabled=False; 
            Kuka.MovementParameterFieldEnabled=False; 
            Kuka.IsAngleEnabled=False; 
            Kuka.PointName=P1; 
            Kuka.FrameData.base_no={self.base_no}; 
            Kuka.FrameData.tool_no={self.tool_no}; 
            Kuka.FrameData.ipo_frame=#BASE; 
            Kuka.isglobalpoint=False; 
            Kuka.MoveDataPtpName=PDAT1; 
            Kuka.MovementDataPdat.apo_mode=#CDIS; 
            Kuka.MovementDataPdat.apo_dist=500; 
            Kuka.MovementData.vel={velocity}; 
            Kuka.MovementData.acc={velocity}; 
            Kuka.MovementData.exax_ign=0; 
            Kuka.VelocityPtp={velocity}; 
            Kuka.BlendingEnabled=False; 
            Kuka.APXEnabled=False; 
            Kuka.CurrentCDSetIndex=0")))'''))
            self.dat_file.append('')
    
        ## Add point
        self.__add2files(newPointSRC,newPointDAT)
        self.__update_last_point(coord)
        self._points.append((coord[0],coord[1],coord[2],
                            rot[0],rot[1],rot[2],marker))

    def add_point_SLIN(self, coord, rot, velocity=1, marker=1,
                       coord_trafo=True):

        if coord_trafo:#  Basis change to robot's universal
            for ind in range(3):
                coord[ind] += self.base_coord[ind]

        if self.object_lims is not None:
            self._check_object_lims(coord)
            if self.last_point_coord is not None:
                assert(self._check_path(coord)),\
                    f'''The linear path crosses the object limits.
                    Point no.: {self.__counter_points}'''

        assert velocity > 0 and velocity <= 2,\
            'Velocity for SLIN cannot be out 0-2 m/s'
        assert len(coord)==3,\
            'Coordinates must be an array with ordered XYZ-Coordinates'
        assert len(rot)==3,\
            'Rotation must be an array with ABC-Rotations data'

        velocity_rel = int(velocity/2*100)  # Relative velocity

        ## SRC file
        newPointSRC = [self.__join(f''';FOLD SLIN P{self.__counter_points} Vel=
        {velocity} m/s CPDAT{self.__counter_points} Tool{self.tool} Base{self.base}  ;
        %(((PE)))'''),
        r';FOLD Parameters ;%{h}',
        self.__join(f''';Params IlfProvider=
        kukaroboter.basistech.inlineforms.movement.spline; 
        Kuka.IsGlobalPoint=False; 
        Kuka.PointName=P{self.__counter_points}; 
        Kuka.BlendingEnabled=False; 
        Kuka.MoveDataName=CPDAT{self.__counter_points}; 
        Kuka.VelocityPath={velocity:.3f}; 
        Kuka.VelocityFieldEnabled=True; 
        Kuka.ColDetectFieldEnabled=True; 
        Kuka.CurrentCDSetIndex=0; 
        Kuka.MovementParameterFieldEnabled=True; 
        IlfCommand=SLIN'''),
        ';ENDFOLD',
        self.__join(f'''SLIN XP{self.__counter_points} WITH $VEL =
         SVEL_CP(2.0, , LCPDAT{self.__counter_points}), 
         $TOOL = STOOL2(FP{self.__counter_points}), 
         $BASE = SBASE(FP{self.__counter_points}.BASE_NO), 
         $IPO_MODE = SIPO_MODE(FP{self.__counter_points}.IPO_FRAME), 
         $LOAD = SLOAD(FP{self.__counter_points}.TOOL_NO), 
         $ACC = SACC_CP(LCPDAT{self.__counter_points}), 
         $ORI_TYPE = SORI_TYP(LCPDAT{self.__counter_points}), 
         $APO = SAPO(LCPDAT{self.__counter_points}), 
         $JERK = SJERK(LCPDAT{self.__counter_points}), 
         $COLLMON_TOL_PRO[1] = USE_CM_PRO_VALUES(0)'''),
        ';ENDFOLD\n']

        ## DAT-File
        newPointDAT = [self.__join(f'''DECL E6POS XP{self.__counter_points}
        =(((X {coord[0]},Y {coord[1]},Z {coord[2]},
        A {rot[0]},B {rot[1]},C {rot[2]},
        S 6,T 16,E1 0.0,
        E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0)))'''),
        self.__join(f'''DECL FDAT FP{self.__counter_points}
        =(((TOOL_NO {self.tool_no},BASE_NO {self.base_no}
        ,IPO_FRAME #BASE,POINT2[] " ")))'''),
        self.__join(f'''DECL LDAT LCPDAT{self.__counter_points}=
        (((VEL {velocity},ACC {velocity_rel}.000,APO_DIST 500.000,APO_FAC 50.0000,
        AXIS_VEL {velocity_rel}.000,AXIS_ACC {velocity_rel}.000,ORI_TYP #VAR,CIRC_TYP #BASE,
        JERK_FAC 50.0000,GEAR_JERK 100.000,
        EXAX_IGN 0)))'''),'']

        if self.__counter_points==1:
            newPointDAT.append(self.__join(f'''DECL MODULEPARAM_T LAST_TP_PARAMS=
            (((PARAMS[] "Kuka.VelocityFieldEnabled=True; 
             Kuka.ColDetectFieldEnabled=True; 
             Kuka.MovementParameterFieldEnabled=True; 
             Kuka.IsAngleEnabled=False; Kuka.PointName=P1; 
             Kuka.FrameData.base_no={self.base_no}; 
             Kuka.FrameData.tool_no={self.tool_no}; 
             Kuka.FrameData.ipo_frame=#BASE; 
             Kuka.isglobalpoint=False; 
             Kuka.MoveDataName=CPDAT1; 
             Kuka.MovementData.apo_fac={velocity_rel}; 
             Kuka.MovementData.apo_dist=500; 
             Kuka.MovementData.axis_acc={velocity_rel}; 
             Kuka.MovementData.axis_vel={velocity_rel}; 
             Kuka.MovementData.circ_typ=#BASE; 
             Kuka.MovementData.jerk_fac=50; 
             Kuka.MovementData.ori_typ=#VAR; 
             Kuka.MovementData.vel={velocity}; 
             Kuka.MovementData.acc={velocity_rel}; 
             Kuka.MovementData.exax_ign=0; 
             Kuka.VelocityPath={velocity}; 
             Kuka.BlendingEnabled=False; 
             Kuka.APXEnabled=False; 
             Kuka.CurrentCDSetIndex=0")))'''))
        
        ## Add point
        self.__add2files(newPointSRC,newPointDAT)
        self.__update_last_point(coord)
        self._points.append((coord[0],coord[1],coord[2],
                            rot[0],rot[1],rot[2],marker))

    def add_point_SCIRC(self, coord_mid, rot_mid, coord, rot, velocity=1,
                        marker=1, coord_trafo=True):

        if coord_trafo:#  Basis change to robot's universal
            for ind in range(3):
                coord[ind] += self.base_coord[ind]
                coord_mid[ind] += self.base_coord[ind]

        if self.object_lims is not None:
            self._check_object_lims(coord)
            self._check_object_lims(coord_mid)
            if self.last_point_coord is not None and not self._check_path(coord):
                print(f'''Warning! The linear path crosses the object limits.
                Point no.: {self.__counter_points}''')
                if not self.__trust:
                    assert(self._check_path(coord)),\
                        'Turn trust mode on for defining such paths.'

        assert velocity > 0 and velocity <= 2,\
            'Velocity for SLIN cannot be out 0-2 m/s'
        assert len(coord_mid)==3,\
            'Coordinates for middle point must be an array with ordered XYZ-Coordinates'
        assert len(coord)==3,\
            'Coordinates must be an array with ordered XYZ-Coordinates'
        assert len(rot)==3,\
            'Rotation must be an array with ABC-Rotations data'
        assert len(rot_mid)==3,\
            'Rotation for middle point must be an array with ABC-Rotations data'
        
        velocity_rel = int(velocity/2*100)

        ## SRC
        newPointSRC = [self.__join(f''';FOLD SCIRC
         P{self.__counter_points} P{self.__counter_points+1} 
         Vel={velocity} m/s CPDAT{self.__counter_points} Tool{self.tool} 
         Base{self.base}  ;%(((PE)))'''),
        ';FOLD Parameters ;%{h}',
        self.__join(f''';Params IlfProvider=
        kukaroboter.basistech.inlineforms.movement.spline; Kuka.IsGlobalPoint=False;
         Kuka.PointName=P{self.__counter_points+1}; 
         Kuka.HelpPointName=P{self.__counter_points}; 
         Kuka.BlendingEnabled=False; 
         Kuka.MoveDataName=CPDAT{self.__counter_points}; 
         Kuka.VelocityPath={velocity}; 
         Kuka.VelocityFieldEnabled=True; 
         Kuka.ColDetectFieldEnabled=True; 
         Kuka.CurrentCDSetIndex=0; 
         Kuka.MovementParameterFieldEnabled=True; 
         IlfCommand=SCIRC'''),
        ';ENDFOLD',
        self.__join(f'''SCIRC XP{self.__counter_points}, XP{self.__counter_points+1} 
           WITH $VEL = SVEL_CP({velocity:.1f}, , LCPDAT{self.__counter_points}), 
         $TOOL = STOOL2(FP{self.__counter_points+1}), 
         $BASE = SBASE(FP{self.__counter_points+1}.BASE_NO), 
         $IPO_MODE = SIPO_MODE(FP{self.__counter_points+1}.IPO_FRAME), 
         $LOAD = SLOAD(FP{self.__counter_points+1}.TOOL_NO), 
         $ACC = SACC_CP(LCPDAT{self.__counter_points}), 
         $ORI_TYPE = SORI_TYP(LCPDAT{self.__counter_points}), 
         $CIRC_TYPE = SCIRC_TYP(LCPDAT{self.__counter_points}), 
         $APO = SAPO(LCPDAT{self.__counter_points}), 
         $CIRC_MODE = SCIRC_M(LCPDAT{self.__counter_points}), 
         $JERK = SJERK(LCPDAT{self.__counter_points}), 
         $COLLMON_TOL_PRO[1] = USE_CM_PRO_VALUES(0)'''),
        ';ENDFOLD']

        ## DAT
        newPointDAT = [self.__join(f'''DECL E6POS 
          XP{self.__counter_points}=(((X {coord_mid[0]},Y {coord_mid[1]},
         Z {coord_mid[2]},A {rot_mid[0]},B {rot_mid[1]},C {rot_mid[2]},
         S 6,T 16,E1 0.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0)))'''),
        self.__join(f'''DECL E6POS 
          XP{self.__counter_points+1}=(((X {coord[0]},Y {coord[1]},Z {coord[2]},
         A {rot[0]},B {rot[1]},C {rot[2]},S 2,T 10,
         E1 0.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0)))'''),
        self.__join(f'''DECL FDAT 
          FP{self.__counter_points+1}=(((TOOL_NO {self.tool_no},
         BASE_NO {self.base_no},IPO_FRAME #BASE,POINT2[] " ")))'''),
        self.__join(f'''DECL LDAT LCPDAT{self.__counter_points}= ((( VEL {velocity},
        ACC {velocity_rel}.000,APO_DIST 500.000,APO_FAC 50.0000,AXIS_VEL {velocity_rel}.000,
        AXIS_ACC {velocity_rel}.000,ORI_TYP #VAR,CIRC_TYP #BASE,JERK_FAC 50.0000,
        GEAR_JERK 100.000,EXAX_IGN 0,CB (((AUX_PT (((ORI #CONSIDER,E1 #CONSIDER,
        E2 #CONSIDER,E3 #CONSIDER,E4 #CONSIDER,E5 #CONSIDER,
        E6 #CONSIDER))),TARGET_PT (((ORI #INTERPOLATE,E1 #INTERPOLATE,
        E2 #INTERPOLATE,E3 #INTERPOLATE,E4 #INTERPOLATE,E5 #INTERPOLATE,
        E6 #INTERPOLATE))) ))) )))''')]

        if self.__counter_points==1:
            newPointDAT.append(self.__join(f'''DECL MODULEPARAM_T LAST_TP_PARAMS=
            (((PARAMS[] "Kuka.VelocityFieldEnabled=True; 
             Kuka.ColDetectFieldEnabled=True; 
             Kuka.MovementParameterFieldEnabled=True; 
             Kuka.IsAngleEnabled=False; 
             Kuka.PointName=P2; 
             Kuka.FrameData.base_no={self.base_no}; 
             Kuka.FrameData.tool_no={self.tool_no}; 
             Kuka.FrameData.ipo_frame=#BASE; 
             Kuka.isglobalpoint=False; 
             Kuka.MoveDataName=CPDAT1; 
             Kuka.MovementData.cb=(((AUX_PT (((ORI #CONSIDER,E1 #CONSIDER,
             E2 #CONSIDER,E3 #CONSIDER,E4 #CONSIDER,E5 #CONSIDER,E6 #CONSIDER))),
             TARGET_PT (((ORI #INTERPOLATE,E1 #INTERPOLATE,E2 #INTERPOLATE,
             E3 #INTERPOLATE,E4 #INTERPOLATE,E5 #INTERPOLATE,E6 #INTERPOLATE))) ))); 
             Kuka.MovementData.apo_fac=50; 
             Kuka.MovementData.apo_dist=500; 
             Kuka.MovementData.axis_acc={velocity_rel}; 
             Kuka.MovementData.axis_vel={velocity_rel}; 
             Kuka.MovementData.circ_typ=#BASE; 
             Kuka.MovementData.jerk_fac=50; 
             Kuka.MovementData.ori_typ=#VAR; 
             Kuka.MovementData.vel={velocity}; 
             Kuka.MovementData.acc={velocity_rel}; 
             Kuka.MovementData.exax_ign=0; 
             Kuka.VelocityPath={velocity}; 
             Kuka.BlendingEnabled=False; 
             Kuka.APXEnabled=False; 
             Kuka.CurrentCDSetIndex=0; 
             Kuka.HelpPointName=P1")))'''))
        
        self.__counter_points+=1

        ## Add point
        self.__add2files(newPointSRC,newPointDAT)
        self.__update_last_point(coord)
        self._points.append((coord[0],coord[1],coord[2],
                            rot[0],rot[1],rot[2],marker))

    def __add2files(self,src:list,dat:list):
        '''Inside function to append commands to files'''
        for n in src:
            self.src_file.append(n)
        for n in dat:
            self.dat_file.append(n)
        self.__counter_points+=1 # Next point

    def spherical2cart(self,point):
        '''
        Inputs:
        --------------
        point: array with (r,phi,theta)
        r is radius, phi is azimuthal and theta the polar angle (rad)

        Returns:
        --------------
        point_cart: same point in cartesian coordinates

        Remarks:
        - Attention: The conversion takes a base into account whose x axis
        shows to from the robot to the outside! 
        - phi in [pi/2,3pi/2] shows to the robot
        - only theta in [0,pi/2] is allowed so far'''
        assert(point[2] <= np.pi/2 and point[2]>= 0),\
            'Theta must be between [0,pi/2]'
        if point[1] < np.pi/2 and point[1] > 3*np.pi/2:
            if self.__trust:
                print('''Warning: Point is behind the object. Special attention
                must be given to the robot's movements.''')
            else:
                assert(self.__trust),\
                    'Point not allowed. Modify __trust parameter to use it'
        x = point[0]*np.cos(point[1])*np.sin(point[2])
        y = point[0]*np.sin(point[1])*np.sin(point[2])
        z = point[0] * np.cos(point[2])
        return [np.around(x,3),np.around(y,3),np.around(z,3)]

    def set_object_lims(self,lims):
        '''Sets the limits of the object to measure. Only the negative
        half of the x-axis is taken into account for the maximum distance, 
        since no measurements are made behind the object.'''
        assert len(lims)==3,'Object limits must be in x, y and z'
        for ind in range(3):
            lims[ind][0] += self.base_coord[ind]
            lims[ind][1] += self.base_coord[ind]
        self.object_lims = []
        self.object_max = 0
        for ind,i in enumerate(lims):
            assert len(i)==2,'There must be two limits in every direction'
            self.object_lims.append(sorted(i))
            if ind==1 or ind==2:
                self.object_max+=((i[0]-i[1])/2)**2
            else:
                self.object_max+=(i[0]-i[1])**2
        self.object_max = self.object_max**0.5

    def _check_object_lims(self,coord,security_check=True):
        '''checks if point is within the limits of the object
        and also if it's nearer than the security distance.'''
        cond = []
        for i in range(3):
            cond.append(coord[i]<self.object_lims[i][1] and \
                coord[i]>self.object_lims[i][0])
        assert (not all(cond)),\
                f'''Given point is inside the object limits.
                Point no.: {self.__counter_points}'''
        if (not self.__trust) and security_check:
                assert np.dot(coord,coord)**0.5 > \
                self.object_max*self.security_factor,\
                    f'''The point is too close to object.
                    Point no.: {self.__counter_points}
                    Min distance: {self.object_max*self.security_factor}
                    Point distance: {np.dot(coord,coord)**0.5}
                    
                    Turn trust mode on if points near the object are needed.'''

    def _check_path(self,coord,accuracy=300):
        '''checks the linear path between the last saved point and a given one.

        It will return False for a linear path that crosses the object limits.

        Accuracy parameter defines how many discrete points are evaluated 
        in between.'''
        temp = self.last_point_coord
        path_points = np.vstack([ np.linspace(temp[0],coord[0],accuracy),
                                 np.linspace(temp[1],coord[1],accuracy),
                                 np.linspace(temp[2],coord[2],accuracy)]).T
        for n in path_points:
            cond = []
            for i in range(3):
                cond.append(n[i]<self.object_lims[i][1] and \
                    n[i]>self.object_lims[i][0])
            inside = all(cond) # True when point is inside, False when not
            if inside:
                break
        return not inside # returns False for a bad path

    def get_counter_points(self):
        return self.__counter_points

    def set_trust(self,mode:bool):
        self.__trust = mode

    def export(self,path='./',save_points=True):
        # Last fixes
        self.dat_file[1] = self.dat_file[1].replace('insertPointNumber',
                                                    str(self.__counter_points-1))
        self.src_file[1] = self.src_file[1].replace('insertPointNumber',
                                                    str(self.__counter_points-1))
        # Export .dat and .src files
        self.dat_file.append(r'ENDDAT')
        with open(path+self.name+'.dat', 'w') as file:
            file.write('\n'.join(self.dat_file))

        self.src_file.append(r';FOLD SPTP HOME Vel=100 % DEFAULT ;%{PE}')
        self.src_file.append(r';FOLD Parameters ;%{h}')
        self.src_file.append(
            self.__join(''';Params IlfProvider=kukaroboter.basistech.inlineforms.movement.spline; 
        Kuka.IsGlobalPoint=False; 
        Kuka.PointName=HOME; Kuka.BlendingEnabled=False; 
        Kuka.MoveDataPtpName=DEFAULT; 
        Kuka.VelocityPtp=100; 
        Kuka.VelocityFieldEnabled=True; 
        Kuka.CurrentCDSetIndex=0; 
        Kuka.MovementParameterFieldEnabled=True; 
        IlfCommand=SPTP'''))
        self.src_file.append(';ENDFOLD')
        self.src_file.append(
            self.__join('''SPTP XHOME WITH $VEL_AXIS[1] = SVEL_JOINT(100.0), 
        $TOOL = STOOL2(FHOME), 
        $BASE = SBASE(FHOME.BASE_NO), 
        $IPO_MODE = SIPO_MODE(FHOME.IPO_FRAME), 
        $LOAD = SLOAD(FHOME.TOOL_NO), 
        $ACC_AXIS[1] = SACC_JOINT(PDEFAULT), 
        $APO = SAPO_PTP(PDEFAULT), 
        $GEAR_JERK[1] = SGEAR_JERK(PDEFAULT), 
        $COLLMON_TOL_PRO[1] = USE_CM_PRO_VALUES(0)'''))
        self.src_file.append(';ENDFOLD\n')
        self.src_file.append('END')
        with open(path+self.name+'.src', 'w') as file:
            file.write('\n'.join(self.src_file))

        if save_points:
            dictio = {f'{ind}':n for ind,n in enumerate(self._points)}
            dictio['README'] = '''Every point number is given as a key
            and the values are an ordered vector with [x,y,z,A,B,C,marker].
            The marker is 1 for normal points and 0 for security points.'''
            savemat(path+self.name+'.mat',dictio)

        ## Export message
        print(f'''\n\tExported {self.name}.SRC and -.DAT Files
        –––––––––––––––––––––––––––––––––––––––
        Tool: {self.tool}
        Base: {self.base}
        No. of points: {self.__counter_points-1}
        Trust mode: {self.__trust}\n''')