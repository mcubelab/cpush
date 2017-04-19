classdef Figure < dynamicprops
%     Example:
%     function obj = plot(obj)
%     Name = 'test';
%     Figures.(Name)=Figure;
%     Figures.(Name).filename = Name;  
%     Figures.(Name).Create(2,1); 
%     Figures.(Name).xData = {[obj.t,obj.t];[obj.t,obj.t]};
%     Figures.(Name).yData = {[obj.u_state{1}(:,1),obj.u_state{1}(:,2)];[obj.u_state{1}(:,3),obj.u_state{1}(:,4)]};
%     Figures.(Name).xLabel={'t(s)';'t(s)'};
%     Figures.(Name).yLabel={'$f_{n1}$ (N)';'$f_{n2}$ (N)'};  
%     Figures.(Name).Color = {['b','r'];['b','r']};
%     Figures.(Name).Title = {Name};               
%     Figures.(Name).Plot2d;
%     Figures.(Name).Save(Foldername);

    properties (Constant)
        font_size  = 15;
        line_size  = 15;
        line_width = 2;
    end
    
    properties
        rows = 1;
        cols = 1;
        filename = 'fig';
        Color = {'b'};
        LineSpecifier = {'-'};
        LegendStr = {'legend'};
        xData = {[0,1];[0,1]};
        yData = {[1,2];[1,2]};
        zData = {[1,2];[1,2]};
        Title = {'Title'};
        xLabel = {'t(s)';'t(s)'};
        yLabel = {'$\theta$ (rad)';'$\dot{\theta}$ (rad/s)'}; 
        zLabel = {'$z_1$ (m)';'$z_2$ (m)'}; 
        Length = 0;
        Fig;
        subFig;
        plot;
    end
    
    methods
        %% Constructor
        function obj = Figure
        end
      
        %% Create Figure
        function obj = Create(obj,rows, cols)
            obj.Fig = figure('Color', 'w', 'OuterPosition', [0, 0, 960, 1080], 'PaperPosition', [0, 0, 11, (6/8)*11]);
            obj.rows = rows;
            obj.cols = cols;
         
            NumFig = obj.rows*obj.cols;
            for lv1=1:NumFig
                subplot(obj.rows,obj.cols,lv1);
                set(gcf,'Renderer','OpenGL');
                set(gca,'visible','on');
            end
%             hold on;
            obj.yData = cell(obj.rows+obj.cols);

        end
        %% Plot Figure 2d
        function obj = Plot2d(obj)
            NumFig = obj.rows*obj.cols;
            if obj.Length==0
                Length_loc = length(obj.xData{1});
            else
                Length_loc = obj.Length;
            end

            for lv1=1:NumFig
                try
                    obj.subFig{lv1} = subplot(obj.rows,obj.cols,lv1);
                    xData_row= obj.xData{lv1};
                    xData_row_size = size(xData_row);
                    xData_length = xData_row_size(1,2);
                    yData_row= obj.yData{lv1};
                    yData_row_size = size(yData_row);
                    yData_length = yData_row_size(1,2);

                    try
                        Color_row = obj.Color{lv1};
                    catch
                    end

                    for lv2=1:yData_length

                        try
                            obj.plot{lv1,lv2} = plot(xData_row(1:Length_loc,lv2), yData_row(1:Length_loc,lv2),'Color', Color_row(:,lv2),'LineWidth', obj.line_width); 
                        catch

                            obj.plot{lv1,lv2} = plot(xData_row(1:Length_loc,lv2), yData_row(1:Length_loc,lv2),'LineWidth', obj.line_width); 
                        end
                      
                        try
%                         title(obj.Title{lv1});
                        catch
                        end
                        grid on;
                        xlabel(obj.xLabel{lv1},'fontsize',obj.font_size,'Interpreter','latex', 'FontSize',obj.font_size);
                        ylabel(obj.yLabel{lv1},'fontsize',obj.font_size,'Interpreter','latex', 'FontSize',obj.font_size);
                        hold on;
                    end
                catch
                end
            end
        end
        %% Scatter plot
        function obj = Scatter(obj)
            obj.Fig = scatter(obj.xData{1},obj.yData{1});
            axis equal;
        end
        
        %% Scatter3 plot
        function obj = Scatter3(obj)
            obj.Fig = scatter3(obj.xData{1},obj.yData{1},obj.zData{1},2);
            axis equal;
            lv1 = 1;
            xlabel(obj.xLabel{lv1},'fontsize',obj.font_size,'Interpreter','latex', 'FontSize',obj.font_size);
            ylabel(obj.yLabel{lv1},'fontsize',obj.font_size,'Interpreter','latex', 'FontSize',obj.font_size);
            zlabel(obj.zLabel{lv1},'fontsize',obj.font_size,'Interpreter','latex', 'FontSize',obj.font_size);
        end
           

        %% Add Legend
        function obj = addLegend(obj)
            NumFig = obj.rows*obj.cols;
            for lv1=1:NumFig
                for lv2=1:length(obj.LegendStr(1,:))
                    subplot(obj.rows,obj.cols,lv1);
                    hold on;
                    LegendEntries{lv2,1} = obj.LegendStr{lv1,lv2};
                end
                LegendEntries
                h=legend(LegendEntries);
                set(h,'Interpreter','latex','FontSize',10);
            end
        end
        %% Save Figure
        function obj = Save(obj, FolderPath)   
            try
                Filename = strcat(FolderPath, '/',obj.filename);
                saveas(obj.Fig,Filename,'epsc');
                save(Filename,'obj');
                [a, MSGID] = lastwarn();
                warning('off', MSGID);
            catch
                saveas(obj.Fig,obj.filename,'epsc');
                save(obj.filename,'obj');
                [a, MSGID] = lastwarn();
                warning('off', MSGID);   
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end