var poiPos; 
            var poiData; 
            var geojsonPLZ;
            var geojsonPOI;
            var geojsonPOI2;

            // Defined view when loading map
            var mymap = L.map('map').setView([47.678, 9.3],11);

            // Gets Distances from plzData-file
            function getDistances(poiData) {
                largestNum = 0
                for (var i = 0; i < Object.keys(plzData.features).length; i++) {
                    plzData.features[i].properties.distance = poiData.features[i].properties.distance
                    if (largestNum < plzData.features[i].properties.distance) {
                        largestNum = plzData.features[i].properties.distance
                    }
                    }
            }

            // Red Color Values
            function getColorA(d) {
                return d >= 20000 ? '#800026' :
                       d >= 17500 ? '#bd0026' :
                       d >= 15000 ? '#e31a1c' :
                       d >= 12500 ? '#fc4e2a' :
                       d >= 10000 ? '#fd8d3c' :
                       d >= 7500  ? '#feb24c' :
                       d >= 5000  ? '#fed976' :
                       d >= 2500  ? '#ffeda0' :
                       d >= 0     ? '#ffffcc' :
                       d = -1     ? '#aaaaaa' :
                                    '#09ed09';
            }

            // Red Color Values
            function getColor(d) {
                return d >= 20000 ? '#bd0026' :
                       d >= 16000 ? '#f03b20' :
                       d >= 12000 ? '#fd8d3c' :
                       d >= 8000 ? '#feb24c' :
                       d >= 4000 ? '#fed976' :
                       d >= 0  ? '#ffffb2' :
                       d = -1     ? '#aaaaaa' :
                                    '#09ed09';
            }

            const mapboxToken = window.mapboxToken;

            // Design of map from mapbox
            L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=sk.eyJ1IjoiaXpiYSIsImEiOiJjbDM4bjJ4cHcwMWZ2M2lueWpnNmtyM2JxIn0.FEO0QszlxCnVYyy8AtOL4A', {
                //id: 'mapbox/light-v11', //Option 1
                id: 'mapbox/streets-v12', //Option 2
                //id: 'mapbox/outdoors-v12',
                //id: 'mapbox/dark-v11', //Option 3
                //id: 'mapbox/satellite-v9',
                //id: 'mapbox/satellite-streets-v12',
                //id: 'mapbox/navigation-day-v1',
                //id: 'mapbox/navigation-night-v1',
                tileSize: 512,
                zoomOffset: -1
            }).addTo(mymap);

            function style(feature) {
                return {
                    fillColor: getColor(feature.properties.distance),
                    weight: 3,
                    opacity: 0.7,
                    color: '#824f3c',
                    dashArray: '',
                    fillOpacity: 0.65
                };
            }

            // Polygon borders highlighting function
            function highlightFeature(e) {  
                var layer = e.target;

                // Style of highlighted area
                layer.setStyle({
                    weight: 7,
                    opacity: 0.9,
                    color: '#32bf8b', // greenish color
                    dashArray: '',
                    fillOpacity: 0.7
                });

                // Brings borders to front
                if (!L.Browser.ie && !L.Browser.opera && !L.Browser.edge) {
                    layer.bringToFront();
                }

                // Update tooltip when hovering above area
                info.update(layer.feature.properties);
            }

            // When not hovering anymore above certain area, reset to default style, update tooltip
            function resetHighlight(e) {
                geojsonEvent.resetStyle(e.target);
                info.update();
            }

            // When click on area, zoom to area
            function zoomToFeature(e) {
                mymap.fitBounds(e.target.getBounds());
            }

            // Adding mouse events to functions
            function onEachFeature(feature, layer) {
                layer.on({
                    mouseover: highlightFeature,
                    mouseout: resetHighlight,
                    click: zoomToFeature
                });
            }

            function addEventToArea() {
                // Adding events to areas
                geojsonEvent = L.geoJson(plzData, {
                    style:style,
                    onEachFeature: onEachFeature
                }).addTo(mymap);
            }

            var info = L.control();

            info.onAdd = function (mymap) {
                this._div = L.DomUtil.create('div', 'info'); // create a div with a class "info"
                this.update();
                return this._div;
            };

            // method that we will use to update the control based on feature properties passed
            info.update = function (props) {
                this._div.innerHTML = '<h4>ZIP-Code:</h4>' +  (props ?
                    '<b>' + props.zip:'ZIP-Code of Area') + '<h4>Distance:</h4>' + (props ? '<b>' + props.distance:'Distance between POIs in Area');
            };

            info.addTo(mymap);

            var legend = L.control({position: 'bottomright'});

            legend.onAdd = function (mymap) {

                var div = L.DomUtil.create('div', 'info legend'),
                    //grades = [0, 2500, 5000, 7500, 10000, 12500, 15000, 17500, 20000],
                    grades = [0, 4000, 8000, 12000, 16000, 20000],
                    labels = [];

                // loop through our density intervals and generate a label with a colored square for each interval
                for (var i = 0; i < grades.length; i++) {
                    div.innerHTML +=
                        '<i style="background:' + getColor(grades[i] + 1) + '"></i> ' +
                        grades[i] + (grades[i + 1] ? '&ndash;' + grades[i + 1] + '<br>' : '+');
                }

                return div;
            };

            legend.addTo(mymap);
            var firstDone = false;
            var twoSets;
            var distanceSetDropDown = document.getElementById("distanceSet");
            var distanceQueryDropDown = document.getElementById("querySet");
            var sliderPOI = document.getElementById("poiVisibility");
            onChange();
            function onChange() {
                var value = distanceSetDropDown.value;
                var query = distanceQueryDropDown.value;
                var text = distanceSetDropDown.options[distanceSetDropDown.selectedIndex].text;
                console.log(value, text);
                console.log("There was a change.")
                
                if (twoSets) {
                    if (sliderPOI.checked == true) {
                        mymap.removeLayer(geojsonPOI);
                        mymap.removeLayer(geojsonPOI2);  
                    }
                    mymap.removeLayer(geojsonPLZ);
                    mymap.removeLayer(geojsonEvent);
                } else if (firstDone) {
                    if (sliderPOI.checked == true) {
                        mymap.removeLayer(geojsonPOI);
                    }
                    mymap.removeLayer(geojsonPLZ);
                    mymap.removeLayer(geojsonEvent);
                }
                
                twoSets = false;
                
                if (value == "DoctorsToDoctors" && query == "AverageRoadDistance") {
                    
                    poiPos = poiPosDoctors;
                    poiData = distanceDoctors;

                } else if (value == "FilteredToFiltered" && query == "AverageRoadDistance") {

                    poiPos = poiPosFiltered;
                    poiData = distanceFiltered;
                    
                } else if (value == "SchoolsToSchools" && query == "AverageRoadDistance") {

                    poiPos = poiPosSchools;
                    poiData = distanceSchools;

                } else if (value == "SchoolsToDoctors" && query == "AverageRoadDistance") {

                    twoSets = true;
                    poiPos = poiPosSchools;
                    poiPos2 = poiPosDoctors;
                    poiData = distanceSchoolDoctor;

                } else if (value == "SchoolsToFiltered" && query == "AverageRoadDistance") {

                    twoSets = true;
                    poiPos = poiPosSchools;
                    poiPos2 = poiPosFiltered;
                    poiData = distanceSchoolToFiltered;

                } else if (value == "DoctorsToFiltered" && query == "AverageRoadDistance") {

                    twoSets = true;
                    poiPos = poiPosDoctors;
                    poiPos2 = poiPosFiltered;
                    poiData = distanceDoctorToFiltered;

                } else if (value == "TwoEl" && query == "AverageRoadDistance") {

                    poiPos = poiPosTwoEl;
                    poiData = distanceTwoEl;

                } else if (value == "TwoElDifZIP" && query == "AverageRoadDistance") {

                    poiPos = poiPosDifZIP;
                    poiData = distanceDifZIP;

                } else if (value == "TwoElMultiPol" && query == "AverageRoadDistance") {

                    poiPos = poiPosMultiPol;
                    poiData = distanceMultiPol;

                } else if (value == "TenEl" && query == "AverageRoadDistance") {

                    poiPos = poiPosTenEl;
                    poiData = distanceTenEl;

                } else if (value == "100El" && query == "AverageRoadDistance") {

                    poiPos = poiPos100El;
                    poiData = distance100El;

                } else if (value == "1000El" && query == "AverageRoadDistance") {

                    poiPos = poiPos1000El;
                    poiData = distance1000El;

                } else if (value == "SchoolsToSchools" && query == "ClosestAverageDistance") {

                    poiPos = poiPosSchools;
                    poiData = distanceClosestSchool;

                } else if (value == "DoctorsToDoctors" && query == "ClosestAverageDistance") {

                    poiPos = poiPosDoctors;
                    poiData = distanceClosestDoctor;

                } else if (value == "FilteredToFiltered" && query == "ClosestAverageDistance") {

                    poiPos = poiPosFiltered;
                    poiData = distanceClosestFiltered;

                } else if (value == "SchoolsToDoctors" && query == "ClosestAverageDistance") {

                    twoSets = true;
                    poiPos = poiPosSchools;
                    poiPos2 = poiPosDoctors;
                    poiData = distanceClosestSchoolToDoctor;

                } else if (value == "SchoolsToFiltered" && query == "ClosestAverageDistance") {

                    twoSets = true;
                    poiPos = poiPosSchools;
                    poiPos2 = poiPosFiltered;
                    poiData = distanceClosestSchoolToFiltered;

                } else if (value == "DoctorsToFiltered" && query == "ClosestAverageDistance") {

                    twoSets = true;
                    poiPos = poiPosDoctors;
                    poiPos2 = poiPosFiltered;
                    poiData = distanceClosestDoctorToFiltered;

                } else if (value == "SchoolsToSchools" && query == "LongestAverageDistance") {

                    poiPos = poiPosSchools;
                    poiData = distanceLongestSchool;

                } else if (value == "DoctorsToDoctors" && query == "LongestAverageDistance") {

                    poiPos = poiPosDoctors;
                    poiData = distanceLongestDoctor;

                } else if (value == "FilteredToFiltered" && query == "LongestAverageDistance") {

                    poiPos = poiPosFiltered;
                    poiData = distanceLongestFiltered;

                } else if (value == "SchoolsToDoctors" && query == "LongestAverageDistance") {

                    twoSets = true;
                    poiPos = poiPosSchools;
                    poiPos2 = poiPosDoctors;
                    poiData = distanceLongestSchoolToDoctor;

                } else if (value == "SchoolsToFiltered" && query == "LongestAverageDistance") {

                    twoSets = true;
                    poiPos = poiPosSchools;
                    poiPos2 = poiPosFiltered;
                    poiData = distanceLongestSchoolToFiltered;

                } else if (value == "DoctorsToFiltered" && query == "LongestAverageDistance") {

                    twoSets = true;
                    poiPos = poiPosDoctors;
                    poiPos2 = poiPosFiltered;
                    poiData = distanceLongestDoctorToFiltered;

                } else if (value == "SchoolsToSchools" && query == "ClosestNodeAverageDistance") {

                    poiPos = poiPosSchools;
                    poiData = distanceClosestNodeSchool;

                } else if (value == "DoctorsToDoctors" && query == "ClosestNodeAverageDistance") {

                    poiPos = poiPosDoctors;
                    poiData = distanceClosestNodeDoctor;

                } else if (value == "FilteredToFiltered" && query == "ClosestNodeAverageDistance") {

                    poiPos = poiPosFiltered;
                    poiData = distanceClosestNodeFiltered;

                } else if (value == "SchoolsToDoctors" && query == "ClosestNodeAverageDistance") {

                    twoSets = true;
                    poiPos = poiPosSchools;
                    poiPos2 = poiPosDoctors;
                    poiData = distanceClosestNodeSchoolToDoctor;

                } else if (value == "SchoolsToFiltered" && query == "ClosestNodeAverageDistance") {

                    twoSets = true;
                    poiPos = poiPosSchools;
                    poiPos2 = poiPosFiltered;
                    poiData = distanceClosestNodeSchoolToFiltered;

                } else if (value == "DoctorsToFiltered" && query == "ClosestNodeAverageDistance") {

                    twoSets = true;
                    poiPos = poiPosDoctors;
                    poiPos2 = poiPosFiltered;
                    poiData = distanceClosestNodeDoctorToFiltered;

                } else {
                    console.log("Remove all Layers")
                    mymap.removeLayer(geojsonPOI)
                    mymap.removeLayer(geojsonPLZ)
                    mymap.removeLayer(geojsonEvent)
                    if (twoSets) {
                        mymap.removeLayer(geojsonPOI2)
                    }
                }

                getDistances(poiData);

                geojsonPLZ = L.geoJson(plzData).addTo(mymap);
                addEventToArea();

                if (firstDone) {
                    addPOItoMap();
                }

                firstDone = true;

            }
            distanceQueryDropDown.onchange = onChange;
            distanceSetDropDown.onchange = onChange;

            function addPOItoMap() {

                if (sliderPOI.checked == true) {
                    geojsonPOI = L.geoJson(poiPos).addTo(mymap);
                    if (twoSets) {
                        geojsonPOI2 = L.geoJson(poiPos2).addTo(mymap);
                    }
                } else {
                    mymap.removeLayer(geojsonPOI);
                    if (twoSets) {
                        mymap.removeLayer(geojsonPOI2);
                    }
                }
            }
