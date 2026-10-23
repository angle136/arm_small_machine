function set_arm_workspace_limits(ax, workspaceRadius)
%SET_ARM_WORKSPACE_LIMITS 一次设置并锁定全角度固定工作空间。
%
% workspaceRadius 已由 calculate_arm_workspace_radius 根据机械链和 URDF 网格
% 计算完成。本函数只在窗口初始化时调用；实时串口刷新不再更新范围或格子。

validateattributes(workspaceRadius, {'numeric'}, ...
    {'real', 'finite', 'positive', 'scalar'}, mfilename, 'workspaceRadius');

fixedLimits = [-workspaceRadius, workspaceRadius];
ax.XLim = fixedLimits;
ax.YLim = fixedLimits;
ax.ZLim = fixedLimits;
ax.XLimMode = 'manual';
ax.YLimMode = 'manual';
ax.ZLimMode = 'manual';
axis(ax, 'equal');
axis(ax, 'vis3d');
grid(ax, 'off');
box(ax, 'off');
ax.Color = [0.97, 0.97, 0.97];
ax.Visible = 'off';
ax.Clipping = 'off';
setappdata(ax, 'ArmWorkspaceRadius', workspaceRadius);
end
