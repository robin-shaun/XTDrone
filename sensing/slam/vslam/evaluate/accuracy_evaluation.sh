#!/bin/bash
i=1
while(( $i<= $1))
do
    #absolute translation error
    evo_ape tum groundtruth.tum orbslam2_syn_$i.tum --pose_relation trans_part -v -a  --save_results ./result/orb_trans_ape$i.zip
    evo_ape tum groundtruth.tum vins_fusion_$i.tum --pose_relation trans_part -v -a  --save_results ./result/vins_trans_ape$i.zip

    #absolute angular error
    evo_ape tum groundtruth.tum orbslam2_syn_$i.tum --pose_relation angle_deg -v -a  --save_results ./result/orb_angle_ape$i.zip
    evo_ape tum groundtruth.tum vins_fusion_$i.tum --pose_relation angle_deg -v -a  --save_results ./result/vins_angle_ape$i.zip    

    #relative translation error
    evo_rpe tum groundtruth.tum orbslam2_syn_$i.tum --pose_relation trans_part --delta 0.5 --delta_unit m -v -a --save_results ./result/orb_trans_rpe$i.zip
    evo_rpe tum groundtruth.tum vins_fusion_$i.tum --pose_relation trans_part --delta 0.5 --delta_unit m -v -a --save_results ./result/vins_trans_rpe$i.zip

    #relative angular error
    evo_rpe tum groundtruth.tum orbslam2_syn_$i.tum --pose_relation angle_deg --delta 0.5 --delta_unit m -v -a --save_results ./result/orb_angle_rpe$i.zip
    evo_rpe tum groundtruth.tum vins_fusion_$i.tum --pose_relation angle_deg --delta 0.5 --delta_unit m -v -a --save_results ./result/vins_angle_rpe$i.zip
    
    let "i++"
done
