<?php

    $temp = $_POST["temp"];
    $impedance = $_POST["impedance"];
    $conn = new mysqli('localhost', 'username', 'password', 'database');
    
    if ($conn->connect_error) 
    {
        echo "connection error, contact webmaster.";
    } 
    else
    {
        if(!empty($temp))
        {
            $sql = "insert into temperature_data values (current_timestamp(),'" . $temp ."') ";
            echo "<br>";
            echo $sql;
            echo "<br>";
            $result = $conn->query($sql);
            
            if($result)
            {
                echo "row inserted<br>";
                echo $temp;
            }
            else
            {
                echo "<strong>error</strong>: cannot insert row<br>";
            }
        }
        else
        {
            echo "<strong>error</strong>: temperature input was empty.<br>";
        }
        
        if(!empty($impedance))
        {
            $sql = "insert into impedance_data values (current_timestamp(),'" . $impedance ."') ";
            echo "<br>";
            echo $sql;
            echo "<br>";
            $result = $conn->query($sql);
            
            if($result)
            {
                echo "row inserted<br>";
                echo $temp;
            }
            else
            {
                echo "<strong>error</strong>: cannot insert row<br>";
            }            
        }
        else
        {
            echo "<strong>error</strong>: impedance input was empty.<br>";
        }
    }
    
    
?>
