import os
from datetime import datetime
from pathlib import Path
import copy
class KUKATranslator:
    """
    Translates a trajectory into KUKA KRL using Inline Forms (ILF).
    This matches the standard KUKA SmartPad UI format, allowing for "Touch Up".


    LDAT: Linear movement data (velocity, acceleration, blending, etc.)
        VEL: Velocity in mm/s
        ACC: Acceleration in percentage of maximum
        APO_DIST: Blending distance in mm
        APO_FAC: Blending factor in percentage
        ORI_TYP: Orientation type (e.g., #VAR, #TOOL)
        CIRC_TYP: Circular movement type (e.g., #BASE, #TOOL)
        JERK_FAC: Jerk factor in percentage
    PDAT: Point movement data (velocity, acceleration, blending, etc.)
        VEL: Velocity in percentage of maximum
        ACC: Acceleration in percentage of maximum
        APO_DIST: Blending distance in mm
        APO_MODE: Blending mode (e.g., #CDIS, #CVEL)
        GEAR_JERK: Jerk for gear movements in percentage
        EMAX_IGN: External Axis Maximum Torque Ignore flag (0.0 or 1.0)
    FDAT: Frame data (tool, base, IPO frame, etc.)
        TOOL_NO: Tool number (integer)
        BASE_NO: Base number (integer)
        IPO_FRAME: Interpolation frame (e.g., #BASE, #TOOL)
        POINT2: Secondary point for circular movements (string, e.g., "P2")
        TQ_STATE: Torque Monitoring (TRUE or FALSE)
    """


    def __init__(self, program_name="layer_test", routine_name="Routine2", tool_id=2, base_id=10):
        self.program_name = program_name
        self.routine_name = routine_name
        self.tool_id = tool_id
        self.base_id = base_id
        self.default_LDAT = {'VEL': 0.2, 'ACC': 100, 'APO_DIST': 5, 'APO_FAC': 50.0, 'ORI_TYP': '#VAR', 'CIRC_TYP': '#BASE', 'JERK_FAC': 50.0}
        self.default_PDAT = {'VEL': 100, 'ACC': 100, 'APO_DIST': 5, 'APO_MODE': '#CDIS', 'GEAR_JERK': 100.0, 'EMAX_IGN': 0.0}
        self.default_FDAT = {'TOOL_NO': 2, 'BASE_NO': 10, 'IPO_FRAME': '#BASE', 'POINT2': '" "', 'TQ_STATE': 'FALSE'}
        self.point_counter = 0  
    
    def LIN(self, XDAT, LDAT, FDAT, C_DIS=True, Translation=None):
        """Generates the ILF and DAT for a LIN movement."""

        """future update: Add support for other orientation calculations (e.g., #TOOL, #BASE) and blending types (e.g., #CVEL). This will involve adjusting the ILF generation to include the appropriate parameters based on the LDAT settings and ensuring that the generated code correctly reflects the desired blending behavior. The current implementation assumes a simple linear movement with blending defined by the velocity, which may not cover all use cases."""
        
        src_code = "$BWDSTART = FALSE\n"
        DAT_code = ""
        point_name = ""
        vel = ""
        tool_name = ""
        base_name = ""
        Ldat_name = ""
        Cont = "CONT " if C_DIS else ""

        if isinstance(LDAT, str):
            src_code += f"LDAT_ACT={LDAT}\n"
            Ldat_name = LDAT
        else:
            DAT_code += f"DECL LDAT LCPDATP{self.point_counter}=" + "{"
            Ldat_name = f"LCPDATP{self.point_counter}"
            for key, value in LDAT.items():
                DAT_code += f"{key} {value},"
            DAT_code = DAT_code.rstrip(',') + "}\n"
            src_code += f"LDAT_ACT=LCPDATP{self.point_counter}\n"
            vel = LDAT['VEL']

        if isinstance(FDAT, str):
            src_code += f"FDAT_ACT={FDAT}\n"
        else:
            DAT_code += f"DECL FDAT FP{self.point_counter}=" + "{"
            for key, value in FDAT.items():
                DAT_code += f"{key} {value},"
            DAT_code = DAT_code.rstrip(',') + "}\n"
            src_code += f"FDAT_ACT=FP{self.point_counter}\n"
            tool_name = f"Tool[{FDAT['TOOL_NO']}]"
            base_name = f"Base[{FDAT['BASE_NO']}]"
        src_code += f"BAS (#CP_PARAMS,{LDAT['VEL']})\n" # Improvement Possibility
        src_code += "SET_CD_PARAMS (0)\n" # Improvement Possibility


        code = "LIN "
        if isinstance(XDAT, str):
            point_name = XDAT
            code += f"{point_name} "
        else:
            if Translation is not None:
                code += Translation+":{"
                for key, value in XDAT.items():
                    code += f"{key} {value:.3f},"
                code = code.rstrip(',') + "} "

            if not (isinstance(FDAT, str) and isinstance(LDAT, str)):
                point_name = f"XP{self.point_counter}"
                code += f"{point_name} "
                type_of_point = "E6POS"
                

                DAT_code += f"DECL {type_of_point} {point_name}=" + "{"
                for key, value in XDAT.items():
                    DAT_code += f"{key} {value:.3f},"
                DAT_code = DAT_code.rstrip(',') + "}\n"
                
            else:
                code +='{'
                for key, value in XDAT.items():
                    code += f"{key} {value:.3f},"
                code = code.rstrip(',') + "} "

        code += "C_DIS\n" if C_DIS else "\n"

        src_code += code

        ilf = f";FOLD LIN {point_name[1:]} {Cont}Vel={vel} m/s {Ldat_name[1:]} {tool_name} {base_name} ;%{{PE}}\n;FOLD Parameters ;%{{h}}\n;Params IlfProvider=kukaroboter.basistech.inlineforms.movement.old; Kuka.IsGlobalPoint=False; Kuka.PointName={point_name[1:]}; Kuka.BlendingEnabled={str(C_DIS).upper()}; Kuka.MoveDataName={Ldat_name[1:]}; Kuka.VelocityPath={vel}; Kuka.CurrentCDSetIndex=0; Kuka.MovementParameterFieldEnabled=True; IlfCommand=LIN\n;ENDFOLD\n"
        if not isinstance(LDAT, str) or not isinstance(FDAT, str):
            self.point_counter += 1  

        src_code = ilf + src_code + "\n;ENDFOLD\n"
        return src_code, DAT_code
    
    def circle_parameter(self, Auxiliary = '', Desired = '', Orientation_type = 'VAR', Circular_movement_type = 'PATH'):
        
        
        param = {
            "Aux": Auxiliary,
            "Des": Desired,
            "Orientation_type": Orientation_type,
            "Circular_movement_type": Circular_movement_type
        }
        return param

    def CIRC(self, circle_parametre, LDAT, FDAT ,C_DIS=True):
        """Generates the ILF and DAT for a CIRC movement."""

        """future update: Add support for other orientation calculations (e.g., #TOOL, #BASE) and circular movement types (e.g., #TOOL). This will involve calculating the intermediate point based on the specified CDAT parameters and adjusting the rotation accordingly. The current implementation assumes a simple circular movement in the base frame with orientation defined by the tool axis, which may not cover all use cases."""

        src_code = "$BWDSTART = FALSE\n"
        DAT_code = ""
        point_name = ""
        vel = ""
        tool_name = ""
        base_name = ""
        Ldat_name = ""
        Cont = "CONT " if C_DIS else ""
        Aux = circle_parametre["Aux"]
        Des = circle_parametre["Des"]
        Orientation_type = circle_parametre["Orientation_type"]
        Circular_movement_type = circle_parametre["Circular_movement_type"]
        if isinstance(LDAT, str):
            src_code += f"LDAT_ACT={LDAT}\n"
            Ldat_name = LDAT
        else:
            DAT_code += f"DECL LDAT LCPDATC{self.point_counter}=" + "{"
            Ldat_name = f"LCPDATC{self.point_counter}"
            for key, value in LDAT.items():
                DAT_code += f"{key} {value},"
            DAT_code = DAT_code.rstrip(',') + "}\n"
            src_code += f"LDAT_ACT=LCPDATC{self.point_counter}\n"
            vel = LDAT['VEL']

        if isinstance(FDAT, str):
            src_code += f"FDAT_ACT={FDAT}\n"
        else:
            DAT_code += f"DECL FDAT FC{self.point_counter}=" + "{"
            for key, value in FDAT.items():
                DAT_code += f"{key} {value},"
            DAT_code = DAT_code.rstrip(',') + "}\n"
            src_code += f"FDAT_ACT=FC{self.point_counter}\n"
            tool_name = f"Tool[{FDAT['TOOL_NO']}]"
            base_name = f"Base[{FDAT['BASE_NO']}]"
        src_code += f"$ORI_TYPE = #{Orientation_type}\n"
        src_code += f"$CIRC_TYPE = #{Circular_movement_type}\n"
        src_code += f"BAS (#CP_PARAMS,{LDAT['VEL']})\n" # Improvement Possibility
        src_code += "SET_CD_PARAMS (0)\n" # Improvement Possibility


        code = "CIRC "
        points = [Aux, Des]
        names = ["Aux", "Des"]
        for i, point in enumerate(points):
            if isinstance(point, str):
                names[i] = point
                code += f"{names[i]} "
            else:
                

                if not (isinstance(FDAT, str) and isinstance(LDAT, str)):
                    point_name = f"XC{names[i]}{self.point_counter}"
                    names[i] = point_name
                    code += f"{point_name}, "
                    type_of_point = "E6POS"

                    DAT_code += f"DECL {type_of_point} {point_name}=" + "{"
                    for key, value in point.items():
                        DAT_code += f"{key} {value:.3f},"
                    DAT_code = DAT_code.rstrip(',') + "}\n"
                    
                else:
                    code +='{'
                    for key, value in point.items():
                        code += f"{key} {value:.3f},"
                    code = code.rstrip(',') + "}, "
        code = code.rstrip(', ') + " "


        code += "C_DIS\n" if C_DIS else "\n"

        src_code += code

        ilf = f";FOLD CIRC {names[0][1:]} {names[1][1:]} {Cont}Vel={vel} m/s {Ldat_name[1:]} {tool_name} {base_name} ;%{{PE}}\n;FOLD Parameters ;%{{h}}\n;Params IlfProvider=kukaroboter.basistech.inlineforms.movement.old; Kuka.IsGlobalPoint=False; Kuka.PointName={point_name[1:]}; Kuka.BlendingEnabled={str(C_DIS).upper()}; Kuka.MoveDataName={Ldat_name[1:]}; Kuka.VelocityPath={vel}; Kuka.CurrentCDSetIndex=0; Kuka.MovementParameterFieldEnabled=True; IlfCommand=LIN\n;ENDFOLD\n"
        if not isinstance(LDAT, str) or not isinstance(FDAT, str):
            self.point_counter += 1  

        src_code = ilf + src_code + "\n;ENDFOLD\n"
        return src_code, DAT_code

    def PTP(self, XDAT, PDAT, FDAT, C_PTP=False, Translation=None):
        """Generates the ILF and DAT for a PTP movement."""
        src_code = "$BWDSTART = FALSE\n"
        DAT_code = ""

        point_name = ""
        vel = ""
        tool_name = ""
        base_name = ""
        Cont = "CONT " if C_PTP else ""
        PDAT_name = ""


        if isinstance(PDAT, str):
            src_code += f"PDAT_ACT={PDAT}\n"
            PDAT_name = PDAT
        else:
            DAT_code += f"DECL PDAT PPDATP{self.point_counter}=" + "{"
            for key, value in PDAT.items():
                DAT_code += f"{key} {value},"
            DAT_code = DAT_code.rstrip(',') + "}\n"
            src_code += f"PDAT_ACT=PPDATP{self.point_counter}\n"
            PDAT_name = f"PPDATP{self.point_counter}"
            vel = PDAT['VEL']

        if isinstance(FDAT, str):
            src_code += f"FDAT_ACT={FDAT}\n"
            fdat_name = FDAT
        else:
            fdat_name = f"FP{self.point_counter}"
            DAT_code += f"DECL FDAT {fdat_name}=" + "{"
            for key, value in FDAT.items():
                DAT_code += f"{key} {value},"
            DAT_code = DAT_code.rstrip(',') + "}\n"
            src_code += f"FDAT_ACT={fdat_name}\n"
            TOOL_NO = FDAT.get('TOOL_NO', self.tool_id[0])
            BASE_NO = FDAT.get('BASE_NO', self.base_id[0])
            tool_name = f"Tool[{TOOL_NO}]"
            base_name = f"Base[{BASE_NO}]"

        src_code += f"BAS (#PTP_PARAMS,{PDAT['VEL']})\n" # Improvement Possibility
        src_code += "SET_CD_PARAMS (0)\n" # Improvement Possibility

        code = "PTP "
        if isinstance(XDAT, str):
            point_name = XDAT
            code += f"{point_name} "
        else:
            if Translation is not None:
                code += Translation+":{"
                for key, value in XDAT.items():
                    code += f"{key} {value:.3f},"
                code = code.rstrip(',') + "} "

            elif not (isinstance(FDAT, str) and isinstance(PDAT, str)):
                point_name = f"XP{self.point_counter}"
                code += f"{point_name} "
                if 'X' in XDAT:
                    type_of_point = "E6POS"
                elif 'A1' in XDAT:
                    type_of_point = "E6AXIS"
                DAT_code += f"DECL {type_of_point} {point_name}=" + "{"
                for key, value in XDAT.items():
                    DAT_code += f"{key} {value:.3f},"
                DAT_code = DAT_code.rstrip(',') + "}\n"
            else:
                code +='{'
                for key, value in XDAT.items():
                    code += f"{key} {value:.3f},"
                code = code.rstrip(',') + "} "

        code += "C_PTP\n" if C_PTP else "\n"
        src_code += code
        ilf = f";FOLD PTP {point_name[1:]} {Cont}Vel={vel} % {fdat_name[1:]} {tool_name} {base_name} ;%{{PE}}\n;FOLD Parameters ;%{{h}}\n;Params IlfProvider=kukaroboter.basistech.inlineforms.movement.old; Kuka.IsGlobalPoint=False; Kuka.PointName={point_name[1:]}; Kuka.BlendingEnabled={str(C_PTP).upper()}; Kuka.MoveDataName={PDAT_name[1:]}; Kuka.VelocityPath={vel}; Kuka.CurrentCDSetIndex=0; Kuka.MovementParameterFieldEnabled=True; IlfCommand=PTP\n;ENDFOLD\n"
        if not isinstance(PDAT, str) or not isinstance(FDAT, str):
            self.point_counter += 1  
        src_code = ilf + src_code + "\n;ENDFOLD\n"
        return src_code, DAT_code
    


    def LDAT(self, VEL=0.2, ACC=100, APO_DIST=5, APO_FAC=50.0, ORI_TYP='#VAR', CIRC_TYP='#BASE', JERK_FAC=50.0):
        """Generates an LDAT structure."""
        ldat = {
            "VEL": VEL,
            "ACC": ACC,
            "APO_DIST": APO_DIST,
            "APO_FAC": APO_FAC,
            "ORI_TYP": ORI_TYP,
            "CIRC_TYP": CIRC_TYP,
            "JERK_FAC": JERK_FAC
        }
        return ldat
    
    def PDAT(self, VEL=100, ACC=100, APO_DIST=5, APO_MODE='#CDIS', GEAR_JERK=100.0, EXAX_IGN=0):
        """Generates a PDAT structure."""
        pdat = {
            "VEL": VEL,
            "ACC": ACC,
            "APO_DIST": APO_DIST,
            "APO_MODE": APO_MODE,
            "GEAR_JERK": GEAR_JERK,
            "EXAX_IGN": EXAX_IGN
        }
        return pdat
    
    def FDAT(self, TOOL_NO=2, BASE_NO=10, IPO_FRAME='#BASE', POINT2='" "', TQ_STATE='FALSE'):
        """Generates an FDAT structure."""
        fdat = {
            "TOOL_NO": TOOL_NO,
            "BASE_NO": BASE_NO,
            "IPO_FRAME": IPO_FRAME,
            "POINT2[]": POINT2,
            "TQ_STATE": TQ_STATE
        }
        return fdat
    
    def E6POS(self, X=None, Y=None, Z=None, A=None, B=None, C=None, S=None, T=None, E1=None, E2=None, E3=None, E4=None, E5=None, E6=None):
        """Generates an E6POS structure."""
        e6pos = {}
        temp = {
            "X": X,
            "Y": Y,
            "Z": Z,
            "A": A,
            "B": B,
            "C": C,
            "S": S,
            "T": T,
            "E1": E1,
            "E2": E2,
            "E3": E3,
            "E4": E4,
            "E5": E5,
            "E6": E6
        }
        for key in temp:
            if temp[key] is not None:
                e6pos[key] = temp[key]
        return e6pos
    
    def AXIS(self, A1=-1000, A2=-90,A3=90, A4=0, A5=-90, A6=0):
        """Generates an AXIS structure."""
        axis = {
            "A1": A1,
            "A2": A2,
            "A3": A3,
            "A4": A4,
            "A5": A5,
            "A6": A6
        }
        return axis
    


    def E6AXIS(self, A1=-1000, A2=-90,A3=90, A4=0, A5=-90, A6=0, E1=0, E2=0,E3=0,E4=0,E5=0,E6=0):
        """Generates an AXIS structure."""
        axis = {
            "A1": A1,
            "A2": A2,
            "A3": A3,
            "A4": A4,
            "A5": A5,
            "A6": A6,
            "E1": E1,
            "E2": E2,
            "E3": E3,
            "E4": E4,
            "E5": E5,
            "E6": E6
        }
        return axis

    
    def _generate_dat_header(self):
        """Generates the header for the .dat file, including external declarations."""

        dat_header = f"""&ACCESS RVP1
&REL 1217
&PARAM WOPTRAJECTORYID = 2
&PARAM WOPTEMPLATENAME = WopCoreTemplatePath
&PARAM DISKPATH = KRC:\R1\Program\Working Paths
DEFDAT  {self.program_name}
;FOLD EXTERNAL DECLARATIONS;%{{PE}}%MKUKATPBASIS,%CEXT,%VCOMMON,%P
;FOLD BASISTECH EXT;%{{PE}}%MKUKATPBASIS,%CEXT,%VEXT,%P
EXT  BAS (BAS_COMMAND  :IN,REAL  :IN )
DECL INT SUCCESS
;ENDFOLD (BASISTECH EXT)
;FOLD USER EXT;%{{E}}%MKUKATPUSER,%CEXT,%VEXT,%P
;Make your modifications here

;ENDFOLD (USER EXT)
;ENDFOLD (EXTERNAL DECLARATIONS)

INT APP_GENNUMBER = 215375
INT APP_ANSWER
INT APP_OFFSET
DECL STATE_T APP_STATE

"""

        return dat_header

    def _generate_src_header(self):
        timestamp = datetime.now().strftime("%Y-%m-%d-%H_%M")
        
        # Mirroring the exact structure provided: INI, CHECK, JobInfo, HomePos
        src_header = f"""&ACCESS RVP1
&REL 1217
&PARAM WOPTRAJECTORYID = 2
&PARAM WOPTEMPLATENAME = WopCoreTemplatePath
&PARAM DISKPATH = KRC:\R1\Program\Working Paths
DEF  {self.program_name} ( )

;FOLD INI
InitWopCoreSrc(#Init_Trajectory)
;ENDFOLD (INI)

;FOLD Manual mode
IF X_Rob_InManu THEN
;Write here the code executed only in manual mode

ENDIF
;ENDFOLD

X_Prod_En_Cours[2]=TRUE

;FOLD PTP PT_LoopPos1 Vel=10 % DEFAULT Tool[1]:Gun Base[0];%{{PE}}
;FOLD Parameters ;%{{h}}
;Params IlfProvider=kukaroboter.basistech.inlineforms.movement.old; Kuka.IsGlobalPoint=False; Kuka.PointName=PT_LoopPos1; Kuka.BlendingEnabled=False; Kuka.MoveDataPtpName=DEFAULT; Kuka.VelocityPtp=10; Kuka.CurrentCDSetIndex=0; Kuka.MovementParameterFieldEnabled=True; IlfCommand=PTP
;ENDFOLD
$BWDSTART = FALSE
PDAT_ACT = PDEFAULT
FDAT_ACT = FPT_LoopPos1
BAS(#PTP_PARAMS, 10.0)
SET_CD_PARAMS (0)
PTP XPT_LoopPos1
;ENDFOLD



;FOLD CHECK DAT-FILE GENERATION NUMBER
  IF APP_GENNUMBER <> 215375 THEN
    LOOP
      MsgQuit("InconsistenceSRCandDAT",,,,1)
      HALT
    ENDLOOP
  ENDIF
;ENDFOLD

;fold Jobinfo
;Job information: {self.program_name}
;Product : 3.1.0.28784
;Date: {timestamp}
;Project name: {self.program_name}
;Author: Alhadi Zidan
;Company: CRITT-TJFU
;Division: CSAMProj
;Comment: Autogenerated CSAM Path
;endfold

; --- GLOBAL BLENDING OVERRIDES ---
$APO.CDIS = 1.0  ; Distance blending radius (mm)
$APO.CVEL = 100  ; Velocity blending (%)
$APO.CORI = 5.0  ; Orientation blending (degrees)
$ADVANCE = 5
"""
        return src_header

    def _write_files(self, trajectory, output_dir):
        dat_path = os.path.join(output_dir, f"{self.program_name}.dat")
        src_path = os.path.join(output_dir, f"{self.program_name}.src")
        with open(dat_path, 'w') as f_dat , open(src_path, 'w') as f_src:
            f_dat.write(self._generate_dat_header())
            f_src.write(self._generate_src_header())
            # go to start position
            xdat = self.E6AXIS(A1=-1000, A2=-90, A3=90, A4=0, A5=-90, A6=0,E2 = -41, E3 = 33)
            pdat = self.PDAT(VEL=100, ACC=100)
            fdat = self.FDAT(TOOL_NO=self.tool_id[0], BASE_NO=self.base_id[0])
            src_code, dat_code  = self.PTP(xdat, pdat, fdat, C_PTP=False)
            f_dat.write(dat_code)
            f_src.write(src_code)

            for i ,traj in enumerate(trajectory):
                f_src.write(f"{self.routine_name}_{i}()\n")
                f_src.write(""";FOLD PTP PT_LoopPos1 Vel=60 % DEFAULT Tool[1]:Gun Base[0];%{{PE}}
;FOLD Parameters ;%{{h}}
;Params IlfProvider=kukaroboter.basistech.inlineforms.movement.old; Kuka.IsGlobalPoint=False; Kuka.PointName=PT_LoopPos1; Kuka.BlendingEnabled=False; Kuka.MoveDataPtpName=DEFAULT; Kuka.VelocityPtp=10; Kuka.CurrentCDSetIndex=0; Kuka.MovementParameterFieldEnabled=True; IlfCommand=PTP
;ENDFOLD
$BWDSTART = FALSE
PDAT_ACT = PDEFAULT
FDAT_ACT = FPT_LoopPos1
BAS(#PTP_PARAMS, 60.0)
SET_CD_PARAMS (0)
PTP XPT_LoopPos1
;ENDFOLD
""")
                # here I should add the changing substrate trajectory
            f_src.write("END\n")

            for i, traj in enumerate(trajectory):
                f_src.write(f"\n; --- Routine for substrate {i} ---\n")
                f_dat.write(f"\n; Data for {self.routine_name}_{i}\n")
                f_src.write(f"DEF {self.routine_name}_{i}()\n")

                
                
                Aux = None
                for point in traj:
                    if point['mode']=='LIN':
                        xdat = self.E6POS(X=point['X'], Y=point['Y'], Z=point['Z'], A=point['A'], B=point['B'], C=point['C'])
                        ldat = self.LDAT(VEL=point['VEL'])
                        fdat = self.FDAT(TOOL_NO=self.tool_id[i], BASE_NO=self.base_id[i])
                        src_code, dat_code  = self.LIN(xdat, ldat, fdat, C_DIS=True)
                        f_dat.write(dat_code)
                        f_src.write(src_code)
                        Aux = None
                    elif point['mode']=='CIRC':
                        if Aux is None:
                            Aux = self.E6POS(X=point['X'], Y=point['Y'], Z=point['Z'], A=point['A'], B=point['B'], C=point['C'])
                        else:
                            Des = self.E6POS(X=point['X'], Y=point['Y'], Z=point['Z'], A=point['A'], B=point['B'], C=point['C'])
                            circle_parametre = self.circle_parameter(Aux, Des, Orientation_type='VAR', Circular_movement_type='PATH')
                            ldat = self.LDAT(VEL=point['VEL'])
                            fdat = self.FDAT(TOOL_NO=self.tool_id[i], BASE_NO=self.base_id[i])
                            src_code, dat_code  = self.CIRC(circle_parametre, ldat, fdat, C_DIS=True)
                            Aux = None
                            f_dat.write(dat_code)
                            f_src.write(src_code)
                            
                    
                    
                f_src.write("END\n;End of routine {i}\n")
                f_dat.write(f";ENDDAT for {self.routine_name}_{i}\n\n")


            

            
            f_dat.write("ENDDAT\n")


    

    def _generate_ilf_wait(self, wait_time):
        """Generates the Inline Form (ILF) for a WAIT SEC command."""
        return f""";FOLD WAIT Time={wait_time} sec;%{{PE}}%R 8.5.16,%MKUKATPBASIS,%CWAIT,%VWAIT,%P 3:{wait_time}
WAIT SEC {wait_time}
;ENDFOLD
"""

    

    


    

    def change_substrates(self, i = 0, trajectory = None):
        """Generates a subroutine for moving between substrates."""
        
        return False



    def generate_programs(self, trajectory, output_dir="."):
        #if not self.change_substrates(trajectory=trajectory[0]):
        #    print("Warning: No substrate change routine defined. Please define the movement between substrates.")
        script_dir = Path(__file__).parent.resolve()
        directory = script_dir / output_dir
        os.makedirs(directory, exist_ok=True)
        self._write_files(trajectory, directory)

    

if __name__ == "__main__":
    import numpy as np
    print("Please run sample_test.py to see the KUKA program generation in action with a sample trajectory.")
    i = 1
    theta0 = 40
    dth = 10

    ranging = np.arange(theta0, 180-theta0, dth) 
    import math
    t = math.asin(-0.5)
    print(t)
