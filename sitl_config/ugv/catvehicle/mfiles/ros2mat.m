function retcode = ros2mat(bagfile)
%ROS2MAT Open a rosbag file and convert it into a mat file.
%   (c) Author: Rahul Kumar Bhadani
%   Copyright (c) 2018 Arizona Board of Regents
%   See copyright file
%
%   ROS2MAT('FILEPATH') parses all the data types and extracts
%   the time series data. These timeseries data are saved as
%   .mat file in the same directory where bag file is located.
%   This function returns an integer specifying success/error code.
%   Return code 1: Success
%   Return code -1: Input file doesn't exist
%   Return code -2: Input file is not a bag file.
%   Return code -3: Input file with a .bag extension is not a proper bag 
%       file.
%   Return code -4: One or more message type is unknown by MATLAB.
%
%   Example:
%        Open a robsbag file and convert it to a mat file
%        Ret = ros2mat('2017-01-01-11-11-11.bag')


    if ((exist(bagfile,'file') == 2))
        
        %Check of the given file has bag file extension
        if( strcmp(bagfile(end-3:end),'.bag') == 0)
            retcode = -2;
            fprintf("\nFile '%s' is not a rosbag file.\n\n",bagfile);
        else
            try
                bag = rosbag(bagfile);
                %Get the list of topics
                topics= bag.AvailableTopics;
                %Retrieve the number of topics.
                num_topics = height(topics);
                bagStruct = struct;
                for i =1:num_topics
                  fprintf("\n%d. Reading %s\n",i, char(topics.Row(i)));
                  %Select the i-th topic from bag container. 
                  try
                      %Save the topic name corresponding to the timeseries data
                      %so as to keep the context/meaning.
                      bagStruct(i).topicname = topics.Row(i);
                      bagStruct(i).topictype = topics.MessageType(i);
                      bagSelect = select(bag, 'Topic', char(topics.Row(i)));
                      fprintf("\nNumber of messages in %s is %f\n\n",char(topics.Row(i)), bagSelect.NumMessages);
                      %Extract the timeseries data and save in a structure member
                      bagStruct(i).data = timeseries(bagSelect);
                  catch ME
                        if(strcmp(ME.identifier, 'robotics:ros:message:NoMatlabClass'))
                            fprintf("MATLAB doesn't know this data type and can't parse it.\n");
                            retcode = -4;
                        end
                  end
                end
                save(strcat(bagfile(1:end-4),'.mat'),'bagStruct');
                retcode = 1;
            catch
                retcode = -3;
                fprintf("\nUnable to read %s, input file may not be a proper bagfile.\n\n",...
                    bagfile);

            end
        end
    else
        retcode = -1;
        fprintf("\nFile '%s' doesn't exist\n\n",bagfile);
    end
end

