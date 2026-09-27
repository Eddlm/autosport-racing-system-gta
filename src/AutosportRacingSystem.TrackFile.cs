using GTA;
using GTA.Math;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.IO;
using System.Xml;

namespace ARS
{
    // Track file writing: serialising a route and its props to Tracks\*.xml, behind the creator's Save Track.
    // Split out of AutosportRacingSystem.cs; the statics it reads (CurrentFile, RouteNodes, NodeHalfWidths, CustomProps) stay there.
    public partial class ARS
    {
        // RouteNodes holds one node per metre, so OrderBy(...).First() sorts the whole list just to take
        // its minimum - once per world prop, synchronously inside a save. Both serialisers only ever want
        // the nearest node, which is a single pass.
        static int NearestRouteNode(Vector3 pos, out float distance)
        {
            int nearest = 0;
            distance = float.MaxValue;
            for (int i = 0; i < RouteNodes.Count; i++)
            {
                float d = pos.DistanceTo(RouteNodes[i]);
                if (d < distance) { distance = d; nearest = i; }
            }
            return nearest;
        }

        public void SaveRoute(string filename)
        {
            if (filename == null || filename == "")
            {
                filename = World.GetStreetName(RouteNodes[0]);
            }

            // The name comes from Game.GetUserInput free text, and the write below throws on any
            // character the filesystem rejects, which OnTick would only log.
            foreach (char invalid in Path.GetInvalidFileNameChars()) filename = filename.Replace(invalid, '_');
            filename = filename.Trim();
            if (filename == "") filename = "New Track";

            if (File.Exists(ScriptsFolder + @"\Tracks\" + filename + ".xml"))
            {
                DateTime today = DateTime.Now;
                filename += " (" + today.Year + today.Month + today.Day + today.Hour + today.Minute + today.Second + ")";
            }


            XmlDocument document = new XmlDocument();



            XmlElement element = document.CreateElement("Data");
            document.AppendChild(element);

            XmlComment c = document.CreateComment("comment");

            c.InnerText = " Flares='204255051' would put flares at the startline.\n The value is actually three RGB values from 000 to 255.\n 255255255 would be white.\n false switches them off. ";
            element.AppendChild(c);

            XmlElement trackside = document.CreateElement("Trackside");
            XmlElement t = document.CreateElement("Model");
            t.InnerText = "prop_wheel_tyre";
            trackside.AppendChild(t);

            t = document.CreateElement("Frecuency");
            t.InnerText = "10";
            trackside.AppendChild(t);


            XmlAttribute isFrozen = document.CreateAttribute("Frozen");
            isFrozen.InnerText = "true";
            trackside.Attributes.Append(isFrozen);

            XmlAttribute flares = document.CreateAttribute("Flares");
            flares.InnerText = "204255051";
            trackside.Attributes.Append(flares);

            element.AppendChild(trackside);



            
            XmlElement route = document.CreateElement("Route");

            XmlElement objects = document.CreateElement("Objects");

            XmlElement name = document.CreateElement("Name");
            name.InnerText = filename;
            element.AppendChild(name);




            UI.ShowSubtitle("Write any tags you want for this track, separated by spaces. Example: rally long");
            XmlElement tags = document.CreateElement("Tags");
            XmlElement tag = document.CreateElement("Tag");
            tag.InnerText = World.GetStreetName(RouteNodes[0]);
            tags.AppendChild(tag);
            tag = document.CreateElement("Tag");
            tag.InnerText = World.GetZoneName(RouteNodes[0]).Replace(" ", "");
            tags.AppendChild(tag);

            string userTags = Game.GetUserInput(32);
            if (userTags != "")
            {
                foreach (string s in userTags.Split(' '))
                {
                    tag = document.CreateElement("Tag");
                    tag.InnerText = s;
                    tags.AppendChild(tag);
                }
            }


            element.AppendChild(tags);




            XmlElement p = null;
            XmlElement info = null;
            int i = 0;
            int W = 5;

            foreach (Vector3 v in RouteNodes)
            {
                p = document.CreateElement("Point");
                

                info = document.CreateElement("X");
                info.InnerText = Math.Round(v.X, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("Y");
                info.InnerText = Math.Round(v.Y, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("Z");
                info.InnerText = Math.Round(v.Z, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("Wide");
                // A node missing from the table keeps the previous width rather than defaulting to zero.
                W = (int)(NodeHalfWidths.ContainsKey(i) ? NodeHalfWidths[i] : W);
                info.InnerText = W.ToString();
                p.AppendChild(info);

                route.AppendChild(p);
                i++;
            }


            CustomProps.Clear();


            foreach (Prop propchecked in World.GetAllProps())
            {
                // FreeCamRide is the editor's own drone and always sits over the route being recorded.
                if (propchecked.IsPersistent && !AutoGeneratedProps.Contains(propchecked) && !StartLineFlares.Contains(propchecked) && FreeCamRide != propchecked)
                {

                    float d;
                    NearestRouteNode(propchecked.Position, out d);
                    if (d < 10) CustomProps.Add(propchecked);
                }
            }

            foreach (Prop prop in CustomProps)
            {
                p = document.CreateElement("Prop");
                info = document.CreateElement("Model");
                info.InnerText = prop.Model.Hash.ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                

                info = document.CreateElement("TextureVariation");
                info.InnerText = Function.Call<int>((Hash)0xE84EB93729C5F36A, prop).ToString();
                ;
                p.AppendChild(info);


                info = document.CreateElement("X");
                info.InnerText = Math.Round(prop.Position.X, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("Y");
                info.InnerText = Math.Round(prop.Position.Y, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("Z");
                info.InnerText = Math.Round(prop.Position.Z, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("RotX");
                info.InnerText = Math.Round(prop.Rotation.X, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("RotY");
                info.InnerText = Math.Round(prop.Rotation.Y, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("RotZ");
                info.InnerText = Math.Round(prop.Rotation.Z, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("IsDynamic");
                info.InnerText = (!prop.FreezePosition).ToString(); 


                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                objects.AppendChild(p);

            }
            document.SelectSingleNode("Data").AppendChild(route);
            document.SelectSingleNode("Data").AppendChild(objects);


            document.Save(ScriptsFolder + @"\Tracks\" + filename + ".xml");

            DisplayHelpTextTimed("Refreshing the track list...", 1000);
            // Deliberately no script yield: this runs from a menu handler, inside the menu pool's process
            // step, and yielding mid-frame from there is what the init path already avoids.
            FillKnownTracks(false);
            RefreshTrackList();
            DisplayHelpTextTimed("~g~" + filename + " ~w~saved to Tracks.", 2000);
        }
    }
}
