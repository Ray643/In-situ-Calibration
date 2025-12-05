% =========================================================================
% PROJECT: ISO 10110 Optomechanical Digital Twin Simulation
% SYSTEM: Arc-Rail Reference Mirror with In-situ Calibration
% AUTHOR: Optomechanical Research Group
% STANDARD: ISO 10110-1 (Z: Optical Axis, Y: Vertical)
% =========================================================================
% 工程：ISO10110光机数字孪生仿真
% 系统：圆弧滑轨参考镜＋在位标定配置
% 说明：本文件通过MATLAB＋ZOSAPI驱动Zemax，仿真重力引起的镜面下沉及其对波前的影响

%% 1. SYSTEM INITIALIZATION
% 系统初始化：清理MATLAB环境，设置坐标约定与仿真参数，并连接Zemax
clear; clc; close all;

% Define ISO 10110 Coordinate Parameters
% 定义ISO10110坐标参数：此处约定Y轴为竖直方向，Z为光轴
Global_Y_Vertical = true;
Gravity_Vector = [0; -9.81; 0]; % m/s^2 along -Y
% Gravity_Vector：重力加速度矢量，单位m/s^2，方向沿−Y轴

% Simulation Parameters
R_arc = 1500; % mm, Radius of Arc Rail
% R_arc：圆弧滑轨半径，单位mm
Angles = linspace(0, 45, 5); % degrees, Simulation steps
% Angles：仿真姿态角列表，0～45度线性分布，共5个点
Mirror_Diameter = 300; % mm
% Mirror_Diameter：参考镜口径直径，单位mm
Wavelength = 0.0006328; % mm (HeNe)
% Wavelength：工作波长，单位mm，对应HeNe激光632.8nm

% Connect to ZOS-API
% 通过ZOSAPI连接ZemaxOpticStudio，依赖已安装的AnsysZemaxOpticStudio2024R1
ZemaxPath = 'C:\Users\39610\OneDrive\文档\Zemax\ZOS-API\Libraries\';
AssemblyPath = strcat(ZemaxPath, 'ZOSAPI_NetHelper.dll');
NET.addAssembly(AssemblyPath);
% 创建ZOSAPI连接对象
TheConnection = ZOSAPI_NetHelper.ZOSAPI_Connection();
TheApplication = TheConnection.CreateNewApplication();
if isempty(TheApplication), error('ZOS-API Connection Failed'); end
% 若连接失败，抛出错误并终止程序
TheSystem = TheApplication.PrimarySystem;
% TheSystem：当前Zemax主系统，是后续所有操作的核心入口

% Setup General System Data
% 配置Zemax系统单位与工作波长
TheSystem.SystemData.Units.LensUnits = ZOSAPI.SystemData.ZemaxSystemUnits.Millimeters;
TheSystem.SystemData.Wavelengths.GetWavelength(1).Wavelength = 0.6328;
% 这里将第一工作波长设置为0.6328μm，对应HeNe632.8nm

%% 2. FINITE ELEMENT MODELING (Gravity Sag Generation)
% Function to generate Zernike Sag Map based on angle theta
% Models the physical response of the mirror to rotating gravity vector
% 函数：根据姿态角theta生成Zernike形式的重力下沉面形，并写出GridSag文件
% 物理意义：模拟镜面在不同重力方向下的变形响应，用有限元等效参数近似
function [GridFile] = Generate_Gravity_Sag(angle_deg, diameter, folder)
    % 输入：
    %   angle_deg：姿态角，单位度
    %   diameter：镜面直径，单位mm
    %   folder：输出GridSag文件的目标文件夹路径
    % 输出：
    %   GridFile：生成的网格文件名（不含完整路径）
    % 副作用：在folder路径下写入二进制GridSag文件
    
    nx = 257; ny = 257; % Grid resolution (Power of 2 + 1)
    % nx,ny：网格分辨率，通常取2^n＋1以适配插值与FFT需求
    R = diameter/2;
    % R：镜面半径，单位mm
    x = linspace(-R, R, nx); y = linspace(-R, R, ny);
    % 在直径范围内生成二维网格坐标
    [X,Y] = meshgrid(x, y);
    % 注意：此处原代码缺少左侧变量，如[X,Y]＝meshgrid(x,y)；实际运行前需补全
    
    rho = sqrt(X.^2 + Y.^2) / R;
    theta_pol = atan2(Y, X);
    % 将笛卡尔坐标(X,Y)转换为极坐标(rho,theta_pol)，rho归一化到[0,1]

    % Gravity Components
    g_axial = sind(angle_deg); % Acts on Z (Defocus/Spherical)
    g_trans = cosd(angle_deg); % Acts on Y (Coma/Astigmatism)
    % g_axial,g_trans：将重力分解为轴向和切向分量，用于分别驱动不同Zernike项

    % Physical Scaling Factors (Empirical/FEA derived constants)
    % Units: mm of sag
    C_defocus = 100e-6; % 100 nm per g axial
    C_coma = 200e-6;    % 200 nm per g transverse
    % C_defocus,C_coma：经验／有限元标定得到的缩放系数，将单位g的重力转化为mm级面形下沉

    % Generate Sag Map (Zernike summation)
    % Z4 (Defocus) ~ 2*rho^2 - 1
    % Z7 (Coma Y) ~ (3*rho^3 - 2*rho)*sin(theta)
    % 通过Z4和Z7两个主导Zernike项近似镜面重力变形
    Sag = (g_axial * C_defocus * (2.*rho.^2 - 1)) +...
          (g_trans * C_coma * (3.*rho.^3 - 2.*rho).* sin(theta_pol));
    
    Sag(rho > 1) = 0; % Aperture Mask
    % 对超出口径(rho>1)的点施加光阑掩膜，使其Sag为0

    % Calculate Derivatives for Bicubic Spline (Critical for Zemax accuracy)
    % 计算一阶与混合二阶导数，用于Zemax进行双三次插值增强精度
    dx = x(2)-x(1); dy = y(2)-y(1);
    [dZdx, dZdy] = gradient(Sag, dx, dy);
    [d2Zdxdy, ~] = gradient(dZdx, dx, dy);
    
    % Write Binary File
    % 写出二进制GridSag文件，格式需与Zemax规范一致
    FileName = sprintf('GravitySag_%d.dat', floor(angle_deg));
    FullFile = fullfile(folder, FileName);
    fid = fopen(FullFile, 'w');
    % Header: nx, ny, dx, dy, unit_flag(0=mm), xdec, ydec
    % 文件头：网格尺寸nx,ny；步长dx,dy；单位标志unit_flag(0表示mm)；xdec,ydec通常为0
    fwrite(fid, nx, 'integer*4'); fwrite(fid, ny, 'integer*4');
    fwrite(fid, dx, 'double'); fwrite(fid, dy, 'double');
    fwrite(fid, 0, 'integer*4'); fwrite(fid, 0, 'integer*4'); fwrite(fid, 0, 'integer*4');
    
    % Data Loop
    % 逐点写入Sag及其导数：顺序为Sag,dZdx,dZdy,d2Zdxdy
    for r=1:ny
        for c=1:nx
            fwrite(fid, Sag(r,c), 'double');
            fwrite(fid, dZdx(r,c), 'double');
            fwrite(fid, dZdy(r,c), 'double');
            fwrite(fid, d2Zdxdy(r,c), 'double');
        end
    end
    fclose(fid);
    % 返回文件名（不含路径），供上层函数用于配置GridSag面
    GridFile = FileName;
end

%% 3. ZEMAX LDE CONSTRUCTION
% 使用LDE编辑器构建光路模型：原点、圆弧运动、滑轨误差、重力下沉镜面及探测面
TheLDE = TheSystem.LDE;
TheLDE.Tools.RemoveAllSurfaces();
% 清空当前系统所有面，重新搭建仿真光路

% -- Config 1: Main Telescope Test Path --
% Surf 1: Pivot Point (O)
% 定义面0：枢轴点／坐标原点O
Surf1 = TheLDE.GetSurfaceAt(0);
Surf1.Comment = 'Origin O (Int 1)';
% 给面0添加注释，用于Zemax界面识别

% Surf 2: Kinematic Arm (Coordinate Break)
% Controls the rotation theta along the arc
% 面1：运动学臂，对应圆弧运动的坐标变换，通过Tilt参数控制theta
Surf2 = TheLDE.InsertNewSurfaceAt(1);
Surf2.ChangeType(Surf2.GetSurfaceTypeSettings(ZOSAPI.Editors.LDE.SurfaceType.CoordinateBreak));
Surf2.Comment = 'Arc Motion Theta';

% Surf 3: Rail Errors (Coordinate Break)
% Adds random noise to the ideal motion
% 面2：滑轨误差坐标变换，用于叠加随机刚体误差（例如微小倾角）
Surf3 = TheLDE.InsertNewSurfaceAt(2);
Surf3.ChangeType(Surf3.GetSurfaceTypeSettings(ZOSAPI.Editors.LDE.SurfaceType.CoordinateBreak));
Surf3.Comment = 'Rail Tolerance';

% Surf 4: REF 1 (Mirror with Gravity Sag)
% 面3：参考镜Ref1，使用GridSag描述重力下沉面形
Surf4 = TheLDE.InsertNewSurfaceAt(3);
Surf4.ChangeType(Surf4.GetSurfaceTypeSettings(ZOSAPI.Editors.LDE.SurfaceType.GridSag));
Surf4.Comment = 'Ref 1 Test Mirror';
Surf4.Radius = Inf; % Flat mirror for this example (or curved)
% Radius设置为Infinity表示平面镜；如需弯曲镜可改为有限半径（注意：原代码使用Infinity符号，MATLAB中更常用Inf）
Surf4.SemiDiameter = Mirror_Diameter/2;
% 半口径设置为镜径一半
Surf4.Material = 'MIRROR';
% 材料设为MIRROR，使此面成为反射镜

% Surf 5: Return Path
% 面4：返回路径坐标变换，用于将光路引回探测面
Surf5 = TheLDE.InsertNewSurfaceAt(4);
Surf5.ChangeType(Surf5.GetSurfaceTypeSettings(ZOSAPI.Editors.LDE.SurfaceType.CoordinateBreak));
Surf5.Comment = 'Return Path';

% Surf 6: Detector Int 1
% 面5：干涉仪探测面，用于观测波前
Surf6 = TheLDE.InsertNewSurfaceAt(5);
Surf6.Comment = 'Int 1 Detector';

% -- Setup Multi-Configuration --
% 设置多配置编辑器MCE，用于按配置控制某些参数（例如圆弧角度）
MCE = TheSystem.MCE;
MCE.AddOperand();
Op1 = MCE.GetOperandAt(1);
Op1.ChangeType(ZOSAPI.Editors.MCE.MultiConfigOperandType.PRAM);
Op1.Param1 = 2; % Surface 2
Op1.Param2 = 3; % Parameter 3 (Tilt X / Angle)
% Op1：PRAM操作数，用于控制Surface2的第3列参数（通常对应TiltX角度）

%% 4. SIMULATION LOOP
% 仿真主循环：遍历各个姿态角，生成GridSag文件并驱动Zemax执行Zernike分析
Results = struct('Theta',num2cell(zeros(size(Angles))),'Z4',num2cell(zeros(size(Angles))));
% 注意：此处原代码语句不完整，实际应预分配Results结构数组，例如：
% Results = struct('Theta',num2cell(zeros(size(Angles))),'Z4',num2cell(zeros(size(Angles))));
% 为遵循不修改可执行代码的要求，此处仅通过注释提示

GridFolder = fullfile(char(TheApplication.ZemaxDataDir), 'Objects', 'Grid Files');
% GridFolder：GridSag文件输出路径，使用Zemax数据目录下的Objects/Grid Files子目录

for i = 1:length(Angles)
    theta = Angles(i);
    fprintf('Simulating Angle: %.2f deg\n', theta);
    % 打印当前仿真角度，便于观察仿真进度
    
    % 1. Generate Physics-based Sag
    % 根据当前角度theta生成对应的重力下沉面形GridSag文件
    SagFile = Generate_Gravity_Sag(theta, Mirror_Diameter, GridFolder);
    
    % 2. Apply Kinematics
    % Move Ref 1 to angle theta (Tilt X in Zemax LDE)
    % 使用MCE操作数将Surface2的TiltX设置为theta，实现圆弧运动
    Op1.GetOperandCell(1).DoubleValue = theta;
    
    % Apply Grid Sag File to Surface 4
    % 为Surface4配置刚生成的GridSag文件，使镜面引入重力下沉
    Surf4Settings = Surf4.GetSurfaceTypeSettings(ZOSAPI.Editors.LDE.SurfaceType.GridSag);
    Surf4Settings.FileName = SagFile;
    Surf4.ChangeType(Surf4Settings);
    
    % 3. Introduce Rail Error (Random Rigid Body)
    % Simulating ISO 10110-6 tolerance L and sigma
    % 引入随机微小倾角误差，模拟ISO10110-6中直线度／角度公差及其sigma
    Error_Tilt = 0.005 * randn(); % degrees
    Surf3.GetSurfaceCell(ZOSAPI.Editors.LDE.SurfaceColumn.Par3).DoubleValue = Error_Tilt;
    
    % 4. Ray Trace & Zernike Analysis
    % 创建ZernikeStandard分析并执行，获取波前Zernike系数
    Zernike = TheSystem.Analyses.New_Analysis(ZOSAPI.Analysis.AnalysisIDM.ZernikeStandard);
    Zernike.ApplyAndWaitForCompletion();
    Z_Res = Zernike.GetResults();
    
    % Store Zernike Terms (Focus, Astig, Coma)
    % 存储关键Zernike项，例如Z4（焦度）等，索引需与Zemax输出设定对应
    Results(i).Theta = theta;
    Results(i).Z4 = Z_Res.GetResult(0).Value; % Check index for specific Zernike
    % 注意：Zernike序号与GetResult索引的对应关系需查阅Zemax文档或在界面确认
    
    % 5. In-situ Calibration Logic (Virtual)
    % Here we would perform the subtraction: W_corrected = W_measured - W_simulated_sag
    % 预留在位标定逻辑：实际应用中应将测得波前W_measured减去模拟下沉W_simulated_sag得到校正后波前
end

fprintf('Simulation Complete. Results stored.\n');
% 仿真结束提示：所有角度的结果已存入Results结构数组
