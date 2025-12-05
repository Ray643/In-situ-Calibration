function BuildFullInterferometerSystem()
    % 初始化 ZOS-API 连接
    import ZOSAPI.*;
    zos = ZOSAPI_Connection();
    if ~zos.IsAlive
        app = zos.CreateNewApplication();
    else
        app = zos.ConnectToApplication();
    end
    
    % 获取主系统并初始化
    TheSystem = app.PrimarySystem;
    TheLDE = TheSystem.LDE;
    TheSystem.New(false); % 清空当前设计
    
    % 设置系统参数 (ISO 10110: Y轴竖直)
    TheSystem.SystemData.Aperture.ApertureValue = 100; % 入瞳直径 100mm
    TheSystem.SystemData.Wavelengths.GetWavelength(1).Wavelength = 0.6328; % HeNe
    
    fprintf('正在构建 13 表面双通仿真系统...\n');
    
    % ---------------------------------------------------------
    % 核心参数定义
    % ---------------------------------------------------------
    R_Arc = 1500;      % 圆弧半径 (mm)
    H_Link = 800;      % Ref1 到 Ref2 的垂直距离 (mm)
    GridSagFile = 'Ref1_ErrorMap.dat'; % 假设的重力形变文件
    
    % ---------------------------------------------------------
    % LDE 表面序列构建 (13个表面)
    % ---------------------------------------------------------
    % 我们需要插入13个表面 (加上OBJ和IMA共15行)
    % 初始只有OBJ, STOP, IMAGE (3行). 我们在STOP(1)后插入。
    
    % 批量插入表面，确保总数正确
    % 目标结构:
    % 0: OBJ
    % 1: STOP (Int 2 Source)
    % 2: CB (Int 2 Pos)
    % 3: CB (Aim to Ref 2)
    % 4: Ref 2 (1st Pass)
    % 5: CB (Linkage to Ref 1)
    % 6: CB (Ref 1 Arc Motion)
    % 7: Ref 1 (Target Mirror)  <-- 核心测试面
    % 8: CB (Undo Arc Motion)
    % 9: CB (Linkage Return)
    % 10: Ref 2 (2nd Pass)
    % 11: CB (Return to Int 2)
    % 12: CB (Int 2 Align)
    % 13: IMAGE (Detector)
    
    surf_count = 13;
    for i = 1:surf_count
        TheLDE.InsertNewSurfaceAt(1); % 总是插在第1个位置，以此类推
    end
    
    % 注意：Zemax插入后顺序会变，最先插入的变成了由下往上。
    % 为简单起见，我们重新获取引用并按顺序设置。
    
    % --- Surface 1: Int 2 Aperture (STOP) ---
    s1 = TheLDE.GetSurfaceAt(1);
    s1.Comment = 'Int 2 Aperture';
    s1.Thickness = 100; 
    
    % --- Surface 2: Coordinate Break (定位 Int 2) ---
    s2 = TheLDE.GetSurfaceAt(2);
    s2.ChangeType(ZOSAPI.Editors.LDE.SurfaceType.CoordinateBreak);
    s2.Comment = 'Locate Int 2';
    % 这里可以设置 Int 2 在竖直导轨上的 Y 坐标
    
    % --- Surface 3: Coordinate Break (瞄准 Ref 2) ---
    s3 = TheLDE.GetSurfaceAt(3);
    s3.ChangeType(ZOSAPI.Editors.LDE.SurfaceType.CoordinateBreak);
    s3.Comment = 'Aim to Ref 2';
    
    % --- Surface 4: Ref 2 (第一遍反射) ---
    s4 = TheLDE.GetSurfaceAt(4);
    s4.ChangeType(ZOSAPI.Editors.LDE.SurfaceType.Standard);
    s4.Material = 'MIRROR';
    s4.Comment = 'Ref 2 (Pass 1)';
    s4.Thickness = -H_Link; % 向上传输到 Ref 1
    
    % --- Surface 5: Coordinate Break (机械联动基准) ---
    s5 = TheLDE.GetSurfaceAt(5);
    s5.ChangeType(ZOSAPI.Editors.LDE.SurfaceType.CoordinateBreak);
    s5.Comment = 'Linkage Base';
    
    % --- Surface 6: Coordinate Break (Ref 1 圆弧运动) ---
    % 这是关键：模拟 theta 角旋转
    s6 = TheLDE.GetSurfaceAt(6);
    s6.ChangeType(ZOSAPI.Editors.LDE.SurfaceType.CoordinateBreak);
    s6.Comment = 'Ref 1 Motion (Theta)';
    % 设置 Tilt About X 为变量或多重结构操作数
    
    % --- Surface 7: Ref 1 (待测反射镜 - Grid Sag) ---
    s7 = TheLDE.GetSurfaceAt(7);
    % 设置为 Grid Sag 面型以加载重力误差
    s7Type = s7.GetSurfaceTypeSettings(ZOSAPI.Editors.LDE.SurfaceType.GridSag);
    s7.ChangeType(s7Type);
    s7.Material = 'MIRROR';
    s7.Comment = 'Ref 1 (Test Surface)';
    s7.Radius = -R_Arc; % 假设是凹面或凸面，符号取决于光线方向
    % 导入.DAT 文件 (假设文件已存在)
    % s7.ImportData.ImportDataFile(GridSagFile); 
    
    % --- Surface 8: Coordinate Break (撤销圆弧运动) ---
    s8 = TheLDE.GetSurfaceAt(8);
    s8.ChangeType(ZOSAPI.Editors.LDE.SurfaceType.CoordinateBreak);
    s8.Comment = 'Undo Motion';
    % 设置 Pickup Solve：跟随 Surface 6 的 Tilt X，比例 -1
    solve = s8.GetSurfaceCell(ZOSAPI.Editors.LDE.SurfaceColumn.Par3).CreateSolveType(ZOSAPI.Editors.SolveType.SurfacePickup);
    solve._S_SurfacePickup.Surface = 6;
    solve._S_SurfacePickup.ScaleFactor = -1;
    solve._S_SurfacePickup.Column = ZOSAPI.Editors.LDE.SurfaceColumn.Par3; % Par3 is Tilt X
    s8.GetSurfaceCell(ZOSAPI.Editors.LDE.SurfaceColumn.Par3).SetSolveData(solve);
    
    % --- Surface 9: Coordinate Break (回程联动) ---
    s9 = TheLDE.GetSurfaceAt(9);
    s9.ChangeType(ZOSAPI.Editors.LDE.SurfaceType.CoordinateBreak);
    s9.Comment = 'Return Linkage';
    s9.Thickness = H_Link; % 回到 Ref 2 高度
    
    % --- Surface 10: Ref 2 (第二遍反射) ---
    s10 = TheLDE.GetSurfaceAt(10);
    s10.ChangeType(ZOSAPI.Editors.LDE.SurfaceType.Standard);
    s10.Material = 'MIRROR';
    s10.Comment = 'Ref 2 (Pass 2)';
    % 这里的厚度需要指向 Int 2
    
    % --- Surface 11: Coordinate Break (回程对准) ---
    s11 = TheLDE.GetSurfaceAt(11);
    s11.ChangeType(ZOSAPI.Editors.LDE.SurfaceType.CoordinateBreak);
    s11.Comment = 'Return Alignment';
    
    % --- Surface 12: Coordinate Break (Int 2 接收位置) ---
    s12 = TheLDE.GetSurfaceAt(12);
    s12.ChangeType(ZOSAPI.Editors.LDE.SurfaceType.CoordinateBreak);
    s12.Comment = 'Int 2 Receiver';
    
    % --- Surface 13: IMAGE (Detector) ---
    s13 = TheLDE.GetSurfaceAt(13);
    s13.Comment = 'Detector Plane';
    
    fprintf('系统构建完成。请检查 Zemax LDE 中的 Pickup 设置。\n');
end