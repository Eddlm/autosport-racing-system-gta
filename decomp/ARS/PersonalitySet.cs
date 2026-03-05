namespace ARS;

public class PersonalitySet
{
	public string Name = "Default";

	public int ProbToUse = 25;

	public string SkillRange = "80,100";

	public string Model = "";

	public PersonalityRivals Rivals = new PersonalityRivals();

	public PersonalityStability Stability = new PersonalityStability();
}
