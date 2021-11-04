const api_url =
  "https://opentdb.com/api.php?amount=10&category=18&difficulty=easy";

async function getAPI(url) {
  const response = await fetch(url);
  let data = await response.json();
  console.log(data.results);
  return data.results;
}

getAPI(api_url);
