function dataLink( data1, data2, ax )
%% Draw a link between the corresponding samples of the 2 data sets.
    n_tot = size(data1,1);
    bfh = figure(4);
    set(bfh,'Color',[1,1,1]);
    set(bfh,'MenuBar','none','Name','','NumberTitle','off');
    set(bfh,'Units','normalized','Position',[0.3,0.2,0.2,0.3])
  
    
    sld = uicontrol('Parent',bfh,...
    'Units','normalized',...
    'FontSize',8,...
    'FontWeight','bold',...
    'position',[0.05,0.5,0.9,0.15],...
    'style','slider',...
    'Callback',@sliderCallBack,...
    'Visible','on');
    
    bye_button = uicontrol('Parent',bfh,...
    'Units','normalized',...
    'FontSize',8,...
    'FontWeight','bold',...
    'position',[0.55,0.05,0.4,0.15],...
    'style','pushbutton',...
    'string','Bye Bye!',...
    'Callback',@closeAll,...
    'Visible','on');

    h = gobjects(0);
    
    function sliderCallBack(src,evt)
        
        ind = floor(src.Value*(n_tot-1))+1;
%         ind = 104;
        
        %plot code
        if isempty(h)
            hold(ax,'on')
            h(1) = plot3(ax,[data1(ind,1),data2(ind,1)], [data1(ind,2),data2(ind,2)], [data1(ind,3),data2(ind,3)],'r');
            h(2) = plot3(ax,[data1(ind,1),data2(ind,1)], [data1(ind,2),data2(ind,2)], [data1(ind,3),data2(ind,3)],'rd','MarkerSize',12);
            hold(ax,'off')
        end
        
        h(1).XData = [data1(ind,1),data2(ind,1)];
        h(1).YData = [data1(ind,2),data2(ind,2)];
        h(1).ZData = [data1(ind,3),data2(ind,3)];
        
        h(2).XData = [data1(ind,1),data2(ind,1)];
        h(2).YData = [data1(ind,2),data2(ind,2)];
        h(2).ZData = [data1(ind,3),data2(ind,3)];
    end

    


    function closeAll(~,~)
        close(bfh)
        delete(h)
    end

end
