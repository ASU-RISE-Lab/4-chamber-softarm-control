function par_set = funcLoadExp2Seg( par_set,exp_case )
fprintf( 'Loading exp. data %d ... \n',exp_case )
switch exp_case
	case 1
		set1=importdata('data_collect_1.txt');
    case 2
        set1=importdata('data_collect_2.txt');
    case 3
        set1=importdata('data_collect_3.txt');
     case 4
        set1=importdata('data_collect_4.txt');
     case 5
        set1=importdata('data_collect_5.txt');
     case 6
        set1=importdata('data_collect_6.txt');
     case 7
        set1=importdata('data_collect_7.txt');
     case 8
        set1=importdata('data_collect_8.txt');
     case 9
        set1=importdata('data_collect_9.txt');
     case 10
        set1=importdata('data_collect_10.txt');
     case 11
        set1=importdata('data_collect_11.txt');
     case 12
        set1=importdata('data_collect_12.txt');
     case 13
        set1=importdata('data_collect_13.txt');
     case 14
        set1=importdata('data_collect_14.txt');
     case 15
        set1=importdata('data_collect_15.txt');
end
    par=[];
    time_set=set1(1:end,1)-set1(1,1);
    fprintf( 'Raw Sample Freq is %d, Std. is %d\n',1/mean(time_set(2:end)-time_set(1:end-1)),1/std(time_set(2:end)-time_set(1:end-1)))
    NoLoad=[];
    ts=timeseries(set1(:,2:end),time_set);
%     Fs=40;T=1/Fs;
    T=par_set.Ts;
    timevec=0:T:time_set(end);
    tsout=resample(ts,timevec);
    resampleData=tsout.Data;
    time_set=[];
    time_set=timevec';
    NoLoad=time_set;NoLoad(:,2:18)=resampleData(:,4:20);

    par.time_stamp = time_set; 
    par.pd_Pa = []; par.pd_Pa(:,1:6) = 1e5 *[resampleData(:,1:3),resampleData(:,7:9)]*0.0689476;
    par.pd_MPa=[]; par.pd_MPa(:,1:6) = par.pd_Pa*1e-6;
    par.pd_psi = []; par.pd_psi(:,1:6) = [resampleData(:,1:3),resampleData(:,7:9)];

    par.pm_Pa = []; par.pm_Pa(:,1:6) = 1e5 *[resampleData(:,4:6),resampleData(:,10:12)]* 0.0689476;
    par.pm_psi = []; par.pm_psi(:,1:6) = [resampleData(:,4:6),resampleData(:,10:12)];
    par.pm_MPa= []; par.pm_MPa(:,1:6) = par.pd_Pa*1e-6;

    par.enco_volts = [];par.enco_volts = resampleData(:,13:16);

    par.rigid_1_pose = []; par.rigid_1_pose(:,1:3) = resampleData(:,17:19);
    par.rigid_1_rot = []; par.rigid_1_rot(:,1:4) = resampleData(:,20:23);

    par.rigid_2_pose = []; par.rigid_2_pose(:,1:3) = resampleData(:,24:26);
    par.rigid_2_rot = []; par.rigid_2_rot(:,1:4) = resampleData(:,27:30);

    par.rigid_3_pose = []; par.rigid_3_pose(:,1:3) = resampleData(:,31:33);
    par.rigid_3_rot = []; par.rigid_3_rot(:,1:4) = resampleData(:,34:37);


switch exp_case
	case 1
		par_set.trial1=par;
    case 2
        par_set.trial2=par;
    case 3
        par_set.trial3=par;
     case 4
        par_set.trial4=par;
     case 5
        par_set.trial5=par;
     case 6
        par_set.trial6=par;
     case 7
        par_set.trial7=par;
     case 8
        par_set.trial8=par;
     case 9
        par_set.trial9=par;
     case 10
        par_set.trial10=par;
     case 11
        par_set.trial11=par;
     case 12
        par_set.trial12=par;
     case 13
        par_set.trial13=par;
     case 14
        par_set.trial14=par;
     case 15
        par_set.trial15=par;
end
end