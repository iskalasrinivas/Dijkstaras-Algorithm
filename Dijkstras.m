clc;
clear all;
%% user inputs
% startpoint
prompt='please enter the startnode in the form [x y]: ';
start_node=input(prompt);
prompt='please enter the endnode in the form[x y]: ';
endnode=input(prompt);
prompt='please enter the resolution/gridsize: ';
res=input(prompt);
%% check if entered start and goal node is valid
while(obspace(floor(start_node(1,1)/res),floor(start_node(1,2)/res),res)==1)
    prompt='the node you entered is in obstacle please enter correct startnode: ';
    start_node=input(prompt);
end
while(obspace(floor(endnode(1,1)/res),floor(endnode(1,2)/res),res)==1)
    prompt='the node you entered is in obstacle please enter correct endnode: ';
    endnode=input(prompt); 
end
while(floor(start_node(1,1)/res)<=0||floor(start_node(1,1)/res)>=250/res || floor(start_node(1,2)/res)<=0 ||floor(start_node(1,2)/res)>=150/res)
    prompt='the startnode is out of bounds, enter start node again: ';
    start_node=input(prompt);
end
while(floor(endnode(1,1)/res)<=0||floor(endnode(1,1)/res)>=250/res || floor(endnode(1,2)/res)<=0 ||floor(endnode(1,2)/res)>=150/res)
    prompt='the endnode is out of bounds, enter end node again: ';
    endnode=input(prompt);
end


%% Algorithm (dijkstras)
node=1;
start_node=[floor(start_node(1,1)/res) floor(start_node(1,2)/res)];
startx=start_node(1,1);
starty=start_node(1,2);
endnode=[floor(endnode(1,1)/res),floor(endnode(1,2)/res)];
visited_arr=zeros(floor(250/res),floor(150/res));
visited_a=[];
cost_arr =inf(floor(250/res),(150/res));
cost_arr(1,1)=0;
parent_arr=cell(floor(250/res),(150/res));
q=[];
q=start_node;
z=start_node;
while(isequal(z,endnode)==0)
    %% marking startnode as visited
    visited_arr(start_node(1),start_node(2))=1;
    [z,q]=pop(q,cost_arr);
    [status,newnode]= moveright(z,res);
    if(status==1)
        value=visited(visited_arr,newnode);
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            % update the parent
            parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
            cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+1;
        else
            if(cost_arr(newnode(1),newnode(2))>cost_arr(z(1),z(2))+1)
               cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+1;
               parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};   
            end
               
        end
    end
    
    [status,newnode]= moveleft(z,res);
    if status==1
        value=visited(visited_arr,newnode);
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
            cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+1;
           
        else
            if(cost_arr(newnode(1),newnode(2))>cost_arr(z(1),z(2))+1)
                cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+1;
                parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
              
            end
       end
         
    end
    
    [status,newnode]= moveup(z,res);
    if status==1
        value=visited(visited_arr,newnode);
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
            cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+1;
            
        else
            if(cost_arr(newnode(1),newnode(2))>cost_arr(z(1),z(2))+1)
                cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+1;
                parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
                
            end
        end
            
    end
    
    [status,newnode]= movedown(z,res);
    if status==1
        value=visited(visited_arr,newnode);
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
            cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+1;
      
            
        else
            if(cost_arr(newnode(1),newnode(2))>cost_arr(z(1),z(2))+1)
                cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+1;
                parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
               
            end
        end
    end
    
    [status,newnode]= movedownright(z,res);
    if status==1
        value=visited(visited_arr,newnode);
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
            cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+sqrt(2);
             
             
         else
            if(cost_arr(newnode(1),newnode(2))>cost_arr(z(1),z(2))+sqrt(2))
                cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+sqrt(2)
                parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
            end
         end
    end
    
    [status,newnode]= moveupright(z,res);
    if status==1
        value=visited(visited_arr,newnode);
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
            cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+sqrt(2);
            
            
        else
            if(cost_arr(newnode(1),newnode(2))>cost_arr(z(1),z(2))+sqrt(2))
                cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+sqrt(2);
                parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
               
            end
        end
    end
    [status,newnode]= movedownleft(z,res);
    if status==1
        value=visited(visited_arr,newnode);
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
            cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+sqrt(2);
           
            
        else
            if( cost_arr(newnode(1),newnode(2))>cost_arr(z(1),z(2))+sqrt(2))
                 cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+sqrt(2);
                 parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
            end
        end
    end
    [status,newnode]= moveupleft(z,res);
    if status==1
        value=visited(visited_arr,newnode);
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
            cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+sqrt(2);
            
        else
            if(cost_arr(newnode(1),newnode(2))>cost_arr(z(1),z(2))+sqrt(2))
                cost_arr(newnode(1),newnode(2))=cost_arr(z(1),z(2))+sqrt(2);
                parent_arr(newnode(1),newnode(2))={[z(1) z(2)]};
               
            end
        end
       
    end
end
%% backtracking
a=endnode;
j=endnode;
while(isequal(j,[startx,starty])~=1)
    temp=parent_arr(j(1),j(2));
    j=cell2mat(temp);
    a=cat(1,a,j);
end
a=fliplr(a');
path=a';
path=path.*res;
[row,col]=find(visited_arr==1);
b=cat(2,row,col);
visited_a=visited_a.*res;
%%
fileID= fopen('nodes.txt','w');
fprintf(fileID,'\t%i\t%i \n',visited_a');
fclose(fileID);
    
% path
fileID= fopen('path.txt','w');
fprintf(fileID,'\t%i\t%i \n',path');
fclose(fileID);


 %% insert
 function [q]= insert(q,newnode)
 q=cat(1,q,newnode);
 end
 
 %% pop
 function [inst,s]= pop(s,cost_arr)
 min=1;
 minx=s(1,1);
 miny=s(1,2);
 for i=1:size(s,1)
     x=s(i,1);
     y=s(i,2);
     if(cost_arr(x,y)<cost_arr(minx,miny))
         min=i;
         minx=x;
         miny=y;
         inst=s(min,:);
              %delete the element from queue
     end
         
 end
 inst=s(min,:);
 s(min,:)=[];
 
 end
%% check wether visited or not
function [value]= visited(visited_arr,newnode)
if(visited_arr(newnode(1),newnode(2))==1)
    value=1;
else
    value=0;
end
end
%% defining obstacle space

function [Cobs]= obspace(x,y,res)
o1= (x-(190/res))^2+(y-(130/res))^2 -(15/res)^2;       %% circle
o2=((x-(140/res))^2)/(15/res)^2 + ((y-(120/res))^2)/(6/res)^2 -1;   %% ellipse
% defining half planes for finding obstacle space
h1= -x+(50/res);
h2= -y+(67.5/res);
h3=  x-(100/res);
h4= y-(112.5/res);
if(h1<=0 && h2<=0 && h3<=0 && h4<=0)
    o3=1;
else
    o3=0;
end
% finding obstacle region for oblique figure
%c1
p1=[125/res,56/res];
p2=[150/res,15/res];
l1= -((p1(1,2)-p2(1,2))*x + (p2(1,1) - p1(1,1))*y + (p1(1,1)*p2(1,2) - p2(1,1)*p1(1,2)));
l2= -y+(15/res);
p3=[173/res,15/res];
p4=[163/res,52/res];
l3= -((p3(1,2)-p4(1,2))*x + (p4(1,1) - p3(1,1))*y + (p3(1,1)*p4(1,2) - p4(1,1)*p3(1,2)));
p5=[125/res,56/res];
l4=-((p4(1,2)-p5(1,2))*x + (p5(1,1) - p4(1,1))*y + (p4(1,1)*p5(1,2) - p5(1,1)*p4(1,2)));
% c2
p6=[170/res,90/res];
l5=((p4(1,2)-p6(1,2))*x + (p6(1,1) - p4(1,1))*y + (p4(1,1)*p6(1,2) - p6(1,1)*p4(1,2)));
p7=[193/res,52/res];
l6=((p6(1,2)-p7(1,2))*x + (p7(1,1) - p6(1,1))*y + (p6(1,1)*p7(1,2) - p7(1,1)*p6(1,2)));
p8=[173/res,15/res];
l7=((p7(1,2)-p8(1,2))*x + (p8(1,1) - p7(1,1))*y + (p7(1,1)*p8(1,2) - p8(1,1)*p7(1,2)));
l8=((p3(1,2)-p4(1,2))*x + (p4(1,1) - p3(1,1))*y + (p3(1,1)*p4(1,2) - p4(1,1)*p3(1,2)));
if((l1<=0 && l2<=0 && l3<=0 && l4<=0)||(l5<=0 && l6<=0 && l7<=0 && l8<=0))
    o4=1;
else
    o4=0;
end

if(o1<=0||o2<=0||o3==1||o4==1)
    Cobs=1;
    r=mod(x,res);
    r1=x-r;
    r2=mod(y,res);
    r3=y-r2;
    for x=r1-res:r1+res
        Cobs=1;
    end
    for y=r3-res:r3+res
        Cobs=1;
    end
    
else
    Cobs=0;
end
end
%for i=1:size(O,1)
%     r=mod(O(i,1),res);
%     r1=O(i,1)-r;
%     r2=mod(O(i,2),res);
%     r3=O(i,2)-r2;
%     map(r3:r3+res,r1:r1+res)=1;
% end
%% actions
function[status,newnode]= moveup(currentnode,res)
if(currentnode(2)~=floor(150/res))
   newnode(1)=currentnode(1);
   newnode(2)=currentnode(2)+1;
   if obspace(newnode(1),newnode(2),res)==0
       status=1;
   else
       newnode=[];
       status=0;
   end
else
newnode=[];
status=0;
      
end
end

function [status,newnode]= movedown(currentnode,res)
if(currentnode(2)~=1)
   newnode(1)=currentnode(1);
   newnode(2)=currentnode(2)-1;
   if obspace(newnode(1),newnode(2),res)==0
       status=1;
   else
       newnode=[];
       status=0;
   end
else
newnode=[];
status=0;
   
end
end
function [status,newnode]= moveleft(currentnode,res)
if(currentnode(1)~=1)
   newnode(1)=currentnode(1)-1;
   newnode(2)=currentnode(2);
   if obspace(newnode(1),newnode(2),res)==0
       status=1;
   else
       newnode=[];
       status=0;
   end
else
 newnode=[];
 status=0;
end
end

function [status,newnode]= moveright(currentnode,res)
if(currentnode(1)~=floor(250/res))
   newnode(1)=currentnode(1)+1;
   newnode(2)=currentnode(2);
   if obspace(newnode(1),newnode(2),res)==0
       status=1;
   else
       newnode=[];
       status=0;
   end
else
newnode=[];
status=0;
end
end

function [status,newnode]= moveupright(currentnode,res)
if(currentnode(1)~=floor(250/res)&&currentnode(2)~=floor(150/res))
   newnode(1)=currentnode(1)+1;
   newnode(2)=currentnode(2)+1;
   if obspace(newnode(1),newnode(2),res)==0
       status=1;
   else
       newnode=[];
       status=0;
    end
else
newnode=[];
status=0;
   
end
end

function [status,newnode]= moveupleft(currentnode,res)
if(currentnode(1)~=1&&currentnode(2)~=floor(150/res))
   newnode(1)=currentnode(1)-1;
   newnode(2)=currentnode(2)+1;
   if obspace(newnode(1),newnode(2),res)==0
       status=1;
   else
       newnode=[];
       status=0;
   end
else
newnode=[];
status=0;
   
end
end

function [status,newnode]= movedownright(currentnode,res)
if(currentnode(1)~=floor(250/res)&&currentnode(2)~=1)
   newnode(1)=currentnode(1)+1;
   newnode(2)=currentnode(2)-1;
   if obspace(newnode(1),newnode(2),res)==0
       status=1;
   else
       newnode=[];
       status=0;
       
   end
else
newnode=[];
status=0;
   
end
end

function [status,newnode]= movedownleft(currentnode,res)
if(currentnode(1)~=1&&currentnode(2)~=1)
   newnode(1)=currentnode(1)-1;
   newnode(2)=currentnode(2)-1;
   if obspace(newnode(1),newnode(2),res)==0
       status=1;
   else
       newnode=[];
       status=0;
   end
else
status=0;
newnode=[];
   
end
end
