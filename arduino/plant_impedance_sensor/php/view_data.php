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
        
        $sql = "select * from temperature_data";
        $sql_i = "select * from impedance_data";
        $result = $conn->query($sql);
        $result_i = $conn->query($sql_i);
        echo "<html> <head>";
        echo "\n<script type=\"text/javascript\" src=\"https://www.gstatic.com/charts/loader.js\">\n</script>\n";
        echo "<script type=\"text/javascript\">\ngoogle.charts.load('current', {'packages':['corechart']});\n</script>\n";
        if($result->num_rows > 0)
            $draw_temp = 1;
        if($result_i->num_rows > 0)
            $draw_impedance = 1;
        
        echo "\n<script type=\"text/javascript\">\n";
        
        if($draw_temp == 1)
            echo "google.charts.setOnLoadCallback(drawTemperature);\n";
        if($draw_impedance == 1)
             echo "google.charts.setOnLoadCallback(drawImpedance);\n"; 
        if($draw_temp == 1)
        {
            echo "function drawTemperature() {\n";
            echo "\tvar data = google.visualization.arrayToDataTable([\n";
            echo "\t['Time', 'Temperature'],\n";
    
            $ct = 0;
          
            while($row = $result->fetch_assoc())
            {
                if($ct == ($result->num_rows-1))
                {
                    
                    echo "\t['" . $row['date'] . "', " . $row['temperature'] . "]\n";
                }
                else
                {
                    echo "\t['" . $row['date'] . "', " . $row['temperature'] . "],\n";   
                }
                $ct++;
            }
    
            echo "\t]);\n";
    
            echo "\tvar options = {\n";
            echo "\t\ttitle: 'Temperature vs. Time',\n";
            echo "\t\tcurveType: 'function',\n";
            echo "\t\tlegend: { position: 'bottom' }\n";
            echo "\t};\n";
    
            echo "\tvar chart = new google.visualization.LineChart(document.getElementById('temperature_chart'));\n";
    
            echo "\tchart.draw(data, options);\n";
            echo "}\n";
        }
        if($draw_impedance == 1)
        {
            echo "function drawImpedance() {\n";
            echo "\tvar data = google.visualization.arrayToDataTable([\n";
            echo "\t['Time', 'Impedance'],\n";
    
            $ct = 0;
          
            while($row = $result_i->fetch_assoc())
            {
                if($ct == ($result_i->num_rows-1))
                {
                    
                    echo "\t['" . $row['date'] . "', " . $row['impedance'] . "]\n";
                }
                else
                {
                    echo "\t['" . $row['date'] . "', " . $row['impedance'] . "],\n";   
                }
                $ct++;
            }
    
            echo "\t]);\n";
    
            echo "\tvar options = {\n";
            echo "\t\ttitle: 'Impedance vs. Time',\n";
            echo "\t\tcurveType: 'function',\n";
            echo "\t\tlegend: { position: 'bottom' }\n";
            echo "\t};\n";
    
            echo "\tvar chart = new google.visualization.LineChart(document.getElementById('impedance_chart'));\n";
    
            echo "\tchart.draw(data, options);\n";
            echo "}\n";
        }
    }
    echo "</script>\n";

    echo "</head>\n";
    echo "<body>\n";
    echo "<table><tr><td>\n";

    if($draw_temp == 1)
    {
        echo "<div id=\"temperature_chart\" style=\"width: 900px; height: 500px\"></div>\n";
    }
    else
    {
        echo "<br>no temperature data recorded yet. <br>";
    }
    echo "</td><td>";
    if($draw_impedance == 1)
    {
        echo "<div id=\"impedance_chart\" style=\"width: 900px; height: 500px\"></div>";
    }
    else
    {
        echo "<br>no impedance data recorded yet. <br>";
    }
    echo "</td></tr></table>\n";
    echo "</body>\n";
    echo "</html>\n";
        
?>
    
    

