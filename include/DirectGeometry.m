function iTj_qi = DirectGeometry(qi, geom_model, JointType)
% DirectGeometry Function 
% inputs: 
% qi : current link position;
% iTj is the constant transformation between the base of the link <i>
% and its follower frame <j>;
% jointType :0 for revolute, 1 for prismatic

% output :
% iTj_qi : transformation between the base of the joint <i> and its follower frame taking 
% into account the actual rotation/translation of the joint qi
T=geom_model(1:4,1:4);

if JointType ~= 0 && JointType ~= 1

    err('Values of 0 or 1 should be put for the linkType parameter')


elseif JointType==0 % rotational joint
    T1 = [cos(qi) -sin(qi) 0 0 ;
             sin(qi) cos(qi) 0 0 ;
             0 0 1 0;
             0 0 0 1] ; 

    iTj_qi = T*T1 ; 

elseif JointType==1 % prismatic joint
    T1 = [1 0 0 0 ;
             0 1 0 0 ;
             0 0 1 qi;
             0 0 0 1] ;

    iTj_qi = T*T1 ;

end


end