<?php
    $draw_temp = 0;
    $draw_impedance = 0;
    
    $conn = new mysqli('localhost', 'username', 'password', 'database');
    
    if ($conn->connect_error) 
    {
        echo "connection error, contact webmaster.";
    } 
    else
    {
        
        $sql = "select * from impedance_data order by date desc limit 1";
        $result = $conn->query($sql);
       
        if($result)
        {
            while($row = $result->fetch_assoc())
            {
                    
                echo $row['date'] . "," . $row['impedance'];
            }
        }
        else
        {
            echo "error with sql query";
        }
    }
       
        
?>
    
    

