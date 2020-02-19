bool wordBreak(string s, vector<string>& wordSet)
{
	if (s.empty())
		return true;
	if (wordSet.empty())
		return false;

	int validEnd = 0;
	vector<bool> dp(s.size() + 1, false);
	dp[0] = true;
	for (int i = 0; i < s.size(); i++)
	{
		if (i == validEnd + 1) 
			return false;
		if (!dp[i]) 
			continue;
		for (auto& word : wordSet)
		{
			int newEnd = i + word.size();
			if (newEnd > s.size()) 
				continue;
			if (memcmp(&s[i], &word[0], word.size()) == 0)
			{
				dp[newEnd] = true;
				validEnd = max(validEnd, newEnd);
			}
		}
	}
	return dp.back();
}